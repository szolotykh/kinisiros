"""Create and safely import portable Kinisi map bundles."""

import io
import json
import os
import re
import tempfile
import zipfile


BUNDLE_FORMAT = "kinisi-map"
BUNDLE_VERSION = 1
MAX_BUNDLE_BYTES = 100 * 1024 * 1024
MAP_EXTENSIONS = (".yaml", ".pgm", ".posegraph", ".data")


class MapBundleError(ValueError):
    """Raised when a map bundle is missing, malformed, or unsafe."""


def safe_map_name(value):
    """Return a filesystem-safe map name or raise MapBundleError."""
    name = str(value or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        raise MapBundleError("Map name may contain only letters, digits, _ and -")
    return name


def create_map_bundle(base_path, name, output_path):
    """Package map-server files and an optional SLAM pose graph into a ZIP."""
    name = safe_map_name(name)
    required = [base_path + ".yaml", base_path + ".pgm"]
    if not all(os.path.isfile(path) for path in required):
        raise MapBundleError("Occupancy map files are incomplete")

    graph_paths = [base_path + ".posegraph", base_path + ".data"]
    graph_exists = [os.path.isfile(path) for path in graph_paths]
    if any(graph_exists) and not all(graph_exists):
        raise MapBundleError("SLAM pose graph files are incomplete")

    manifest = {
        "format": BUNDLE_FORMAT,
        "version": BUNDLE_VERSION,
        "name": name,
        "posegraph": all(graph_exists),
    }
    with zipfile.ZipFile(output_path, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("manifest.json", json.dumps(manifest, indent=2))
        for path in required + (graph_paths if all(graph_exists) else []):
            archive.write(path, arcname=os.path.basename(path))
    return manifest


def install_map_bundle(payload, maps_dir, require_posegraph=False):
    """Validate and safely install a Kinisi map bundle into maps_dir."""
    if not payload or len(payload) > MAX_BUNDLE_BYTES:
        raise MapBundleError("Map bundle is empty or too large")
    try:
        archive = zipfile.ZipFile(io.BytesIO(payload), "r")
    except (zipfile.BadZipFile, OSError) as exc:
        raise MapBundleError("Map bundle is not a valid ZIP file") from exc

    with archive:
        infos = archive.infolist()
        names = [info.filename for info in infos]
        if len(names) != len(set(names)):
            raise MapBundleError("Map bundle contains duplicate files")
        if any(
            name != os.path.basename(name) or "/" in name or "\\" in name
            for name in names
        ):
            raise MapBundleError("Map bundle contains an unsafe path")
        if sum(info.file_size for info in infos) > MAX_BUNDLE_BYTES:
            raise MapBundleError("Expanded map bundle is too large")
        if "manifest.json" not in names:
            raise MapBundleError("Map bundle has no manifest")

        try:
            manifest = json.loads(archive.read("manifest.json"))
        except (json.JSONDecodeError, UnicodeDecodeError, KeyError) as exc:
            raise MapBundleError("Map bundle manifest is invalid") from exc
        if manifest.get("format") != BUNDLE_FORMAT:
            raise MapBundleError("Unsupported map bundle format")
        if manifest.get("version") != BUNDLE_VERSION:
            raise MapBundleError("Unsupported map bundle version")
        name = safe_map_name(manifest.get("name"))

        expected = {"manifest.json", name + ".yaml", name + ".pgm"}
        graph_files = {name + ".posegraph", name + ".data"}
        present_graph = graph_files.intersection(names)
        if present_graph and present_graph != graph_files:
            raise MapBundleError("SLAM pose graph files are incomplete")
        has_posegraph = present_graph == graph_files
        if bool(manifest.get("posegraph", False)) != has_posegraph:
            raise MapBundleError("Map bundle manifest does not match its files")
        if require_posegraph and not has_posegraph:
            raise MapBundleError(
                "This map has no SLAM pose graph and cannot resume mapping"
            )
        expected.update(present_graph)
        unexpected = set(names).difference(expected)
        if unexpected:
            raise MapBundleError("Map bundle contains unexpected files")
        if not {name + ".yaml", name + ".pgm"}.issubset(names):
            raise MapBundleError("Occupancy map files are incomplete")

        os.makedirs(maps_dir, exist_ok=True)
        installed = [name + ext for ext in MAP_EXTENSIONS if name + ext in names]
        with tempfile.TemporaryDirectory(dir=maps_dir) as staging:
            for filename in installed:
                target = os.path.join(staging, filename)
                with open(target, "wb") as output:
                    output.write(archive.read(filename))
            backups, published = [], []
            try:
                for filename in installed:
                    destination = os.path.join(maps_dir, filename)
                    if os.path.exists(destination):
                        os.replace(destination, os.path.join(staging, filename + '.previous'))
                        backups.append(filename)
                    os.replace(os.path.join(staging, filename), destination)
                    published.append(filename)
            except OSError:
                for filename in published:
                    os.unlink(os.path.join(maps_dir, filename))
                for filename in backups:
                    os.replace(os.path.join(staging, filename + '.previous'),
                               os.path.join(maps_dir, filename))
                raise

    return {
        "name": name,
        "base": os.path.join(maps_dir, name),
        "occupancy": True,
        "posegraph": has_posegraph,
    }
