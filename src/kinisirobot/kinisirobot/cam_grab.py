#!/usr/bin/env python3
"""Capture one frame from the IMX708 via unicam (/dev/video0) and save a JPEG.

Configures the sensor + unicam capture node, grabs one RG10 (unpacked 10-bit
Bayer RGGB) frame, applies simple gray-world white balance + gamma, demosaics
with a fast half-resolution NumPy binning, and writes a JPEG via PIL. Uses
NumPy+PIL rather than OpenCV: `import cv2` alone costs ~1.6 s per process on the
Pi (dominating capture time), whereas numpy+PIL import in ~0.5 s. Requires the
video group (no sudo).
"""
import subprocess, sys, os, fcntl, struct, numpy as np
from PIL import Image

MEDIA = os.environ.get("MEDIA", "/dev/media0")
VIDEO = os.environ.get("VIDEO", "/dev/video0")
META = os.environ.get("META", "/dev/video1")
SENSOR = os.environ.get("SENSOR", "imx708_wide_noir")
RAW = "/tmp/frame.raw"
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/cam.jpg")
# IMX708 modes: 4608x2592 (full), 2304x1296 (binned). Binned = faster/less noise.
W = int(os.environ.get("W", "2304"))
H = int(os.environ.get("H", "1296"))
BAYER = os.environ.get("BAYER", "RG")  # top-left sample = Red (RGGB)
EMBED = int(os.environ.get("EMBED", "28800"))  # sensor embedded-data width
JPEG_QUALITY = int(os.environ.get("JPEG_QUALITY", "85"))



def run(cmd):
    print("+", " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stdout + r.stderr)
    return r.returncode


def fix_meta_buffersize(dev, size):
    """S_FMT the embedded metadata node so its buffersize matches the sensor's
    embedded pad width; otherwise unicam pipeline validation fails (-EINVAL)."""
    # VIDIOC_S_FMT = _IOWR('V', 5, struct v4l2_format[208]).
    VIDIOC_S_FMT = (3 << 30) | (208 << 16) | (ord('V') << 8) | 5
    V4L2_BUF_TYPE_META_CAPTURE = 13
    SENS = ord('S') | (ord('E') << 8) | (ord('N') << 16) | (ord('S') << 24)
    buf = bytearray(208)
    struct.pack_into('<I', buf, 0, V4L2_BUF_TYPE_META_CAPTURE)  # type
    struct.pack_into('<I', buf, 8, SENS)                        # meta.dataformat
    struct.pack_into('<I', buf, 12, size)                       # meta.buffersize
    with open(dev, 'rb') as f:
        fcntl.ioctl(f, VIDIOC_S_FMT, buf, True)
    got = struct.unpack_from('<I', buf, 12)[0]
    print(f"  meta {dev} buffersize -> {got}")
    return got


def main():
    # 1) Configure sensor source pad + capture node.
    if run(["media-ctl", "-d", MEDIA, "--set-v4l2",
            f'"{SENSOR}":0[fmt:SRGGB10_1X10/{W}x{H}]']):
        print("WARN: could not set binned mode; falling back to full 4608x2592")
        globals()["W"], globals()["H"] = 4608, 2592
        run(["media-ctl", "-d", MEDIA, "--set-v4l2",
             f'"{SENSOR}":0[fmt:SRGGB10_1X10/{W}x{H}]'])
    run(["v4l2-ctl", "-d", VIDEO,
         f"--set-fmt-video=width={W},height={H},pixelformat=RG10"])

    # 1b) Match the embedded-metadata node buffersize to the sensor's embedded
    # pad width, or unicam's pipeline validation rejects STREAMON with -EINVAL.
    fix_meta_buffersize(META, EMBED)

    # 1c) Manual exposure/gain (unicam is raw-only: no auto-exposure loop).
    # vertical_blanking must be raised first — it extends the frame length and
    # thus the allowed exposure maximum. Then set exposure/gains and focus.
    SUBDEV = os.environ.get("SUBDEV", "/dev/v4l-subdev0")
    VCM = os.environ.get("VCM", "/dev/v4l-subdev1")
    VBLANK = os.environ.get("VBLANK", "8000")
    EXPOSURE = os.environ.get("EXPOSURE", "8000")
    AGAIN = os.environ.get("AGAIN", "400")
    DGAIN = os.environ.get("DGAIN", "1024")
    FOCUS = os.environ.get("FOCUS")  # optional VCM focus_absolute (0..1023)
    run(["v4l2-ctl", "-d", SUBDEV, "-c", f"vertical_blanking={VBLANK}"])
    run(["v4l2-ctl", "-d", SUBDEV, "-c",
         f"exposure={EXPOSURE},analogue_gain={AGAIN},digital_gain={DGAIN}"])
    if FOCUS is not None:
        run(["v4l2-ctl", "-d", VCM, "-c", f"focus_absolute={FOCUS}"])

    # 2) Capture frames. Each frame is long (frame length = H + VBLANK lines),
    # so the warm-up count dominates capture time. Manual exposure/fixed focus
    # mean the sensor yields a clean frame quickly; a couple of warm-up frames
    # just flush stale/partial unicam buffers. FRAMES is env-tunable (min 1);
    # --stream-to appends every frame and we keep only the last one below.
    frames = max(1, int(os.environ.get("FRAMES", "2")))
    if run(["v4l2-ctl", "-d", VIDEO, "--stream-mmap",
            f"--stream-count={frames}", f"--stream-to={RAW}"]):
        print("ERROR: capture failed"); return 1

    # v4l2-ctl --stream-to appends every frame; keep only the last full frame.
    frame_bytes = W * H * 2
    total = os.path.getsize(RAW)
    with open(RAW, "rb") as f:
        f.seek(max(0, total - frame_bytes))
        buf = f.read(frame_bytes)
    raw = np.frombuffer(buf, dtype="<u2").reshape(H, W).astype(np.float32)

    # Fast half-resolution demosaic: each 2x2 Bayer block becomes one output
    # pixel (R and B sampled directly, G averaged from its two greens). This is
    # a NumPy-only alternative to cv2.cvtColor and yields a WxH/2 image, which
    # is plenty for a monitoring view and encodes/transfers much faster.
    # BAYER (cv2-style code) gives the 2x2 layout; "RG" == RGGB (verified to
    # match cv2.COLOR_BayerRG2BGR channel-for-channel on this sensor).
    _PATTERNS = {
        "RG": (("R", "G"), ("G", "B")),
        "BG": (("B", "G"), ("G", "R")),
        "GR": (("G", "R"), ("B", "G")),
        "GB": (("G", "B"), ("R", "G")),
    }
    pat = _PATTERNS.get(BAYER, _PATTERNS["RG"])
    pos = {}
    greens = []
    for i in (0, 1):
        for j in (0, 1):
            c = pat[i][j]
            (greens.append((i, j)) if c == "G" else pos.setdefault(c, (i, j)))
    (ri, rj), (bi, bj) = pos["R"], pos["B"]
    (g0i, g0j), (g1i, g1j) = greens
    r = raw[ri::2, rj::2]
    b = raw[bi::2, bj::2]
    g = (raw[g0i::2, g0j::2] + raw[g1i::2, g1j::2]) * 0.5
    rgb = np.stack([r, g, b], axis=-1) / 1023.0   # 10-bit -> [0,1]

    # 3) Gray-world white balance.
    means = rgb.reshape(-1, 3).mean(axis=0) + 1e-6
    gray = means.mean()
    rgb *= (gray / means)
    rgb = np.clip(rgb, 0, 1)

    # 4) Gamma (linear sensor -> sRGB-ish) + mild brightness.
    rgb = np.clip(rgb * 1.1, 0, 1) ** (1 / 2.2)

    out8 = (rgb * 255.0 + 0.5).astype(np.uint8)
    im = Image.fromarray(out8, "RGB")

    # Camera is mounted upside down: rotate the frame. ROTATE env var accepts
    # 0/90/180/270 (degrees clockwise); default 180 to flip the inverted mount.
    rot = int(os.environ.get("ROTATE", "180"))
    _rotmap = {90: Image.ROTATE_270,   # PIL rotates counter-clockwise
               180: Image.ROTATE_180,
               270: Image.ROTATE_90}
    if rot in _rotmap:
        im = im.transpose(_rotmap[rot])

    im.save(OUT, "JPEG", quality=JPEG_QUALITY)
    print(f"OK: wrote {OUT} ({im.width}x{im.height}) bayer={BAYER}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
