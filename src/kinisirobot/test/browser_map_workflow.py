"""Opt-in Playwright smoke test against a running robot web client.

Requires a backup bundle of the currently loaded map. This test resets the
working map, reloads that backup, and finishes with STOP. Run only on a robot
prepared for testing; loading a map may start guarded localization movements.
Screenshots and the JSON report go to --output, not the repository.
"""

import argparse
import json
import math
import re
import time
from pathlib import Path

from playwright.sync_api import sync_playwright, expect


def observe_mapping(page, url, report, output, seconds):
    """Require verified localization, then catch stationary pose or map divergence."""
    deadline = time.monotonic() + 120
    observations = report['mapping_observations'] = []
    waiting = report['localization_observations'] = []
    while True:
        data = page.request.get(url + '/map', timeout=10000).json()
        waiting.append(data['localization'])
        if len(waiting) % 10 == 1:
            print(json.dumps(data['localization']), flush=True)
        if data['map_mode'] in ('localized', 'updating'):
            break
        assert data['map_mode'] != 'localization_error', data['localization']
        assert time.monotonic() < deadline, data['localization']
        page.wait_for_timeout(1000)
    report['checks']['localization_completes'] = True
    assert page.request.post(url + '/estop', timeout=10000).ok
    # Allow the first regenerated grid to arrive before choosing a baseline.
    page.wait_for_timeout(5000)
    started = time.monotonic()
    baseline = None
    while True:
        data = page.request.get(url + '/map', timeout=10000).json()
        row = dict(elapsed=round(time.monotonic() - started, 1),
                   phase=data['map_mode'], pose=data.get('pose'),
                   width=data.get('width'), height=data.get('height'),
                   occupied=sum(value >= 65 for value in data.get('data', [])))
        observations.append(row)
        print(json.dumps(row), flush=True)
        assert row['phase'] in ('localized', 'updating'), data['localization']
        assert row['pose'] and row['occupied'] > 0, row
        if baseline is None:
            baseline = row
        # These are stationary integration-test limits, not navigation tolerances.
        assert abs(row['width'] - baseline['width']) <= 2, row
        assert abs(row['height'] - baseline['height']) <= 2, row
        assert abs(row['occupied'] - baseline['occupied']) <= max(20, baseline['occupied'] * .05), row
        pose, prior = row['pose'], baseline['pose']
        assert math.hypot(pose['x'] - prior['x'], pose['y'] - prior['y']) < .10, row
        yaw_delta = math.atan2(math.sin(pose['yaw'] - prior['yaw']),
                               math.cos(pose['yaw'] - prior['yaw']))
        assert abs(yaw_delta) < math.radians(5), row
        if time.monotonic() - started >= seconds:
            break
        page.wait_for_timeout(2000)
    page.screenshot(path=str(output / 'stable-map.png'), full_page=True)
    report['checks']['stationary_map_stable'] = True


def run(url, bundle, output, observe_seconds=90, allow_motion=False):
    """Exercise real UI controls and record results without sending drive keys."""
    output.mkdir(parents=True, exist_ok=True)
    report = {"checks": {}, "browser_errors": [], "dialogs": []}
    reset_started = False
    map_restored = False
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(viewport={"width": 1440, "height": 1050})
        page.on("pageerror", lambda error: report["browser_errors"].append(str(error)))
        def accept_dialog(dialog):
            """Keep error alerts in the report as well as accepting confirmations."""
            report['dialogs'].append(dialog.message)
            dialog.accept()
        page.on("dialog", accept_dialog)
        try:
            # Heartbeats are continuous, so networkidle is not a useful ready signal.
            page.goto(url, wait_until="domcontentloaded", timeout=20000)
            assert page.request.post(url + '/estop', timeout=10000).ok
            expect(page.locator("#mapphase")).not_to_have_text("Connecting to robot", timeout=15000)
            report["initial_status"] = page.locator("#mapprogress").inner_text()
            page.screenshot(path=str(output / "initial.png"), full_page=True)
            assert page.locator("#localizationmove").count() == 0
            report["checks"]["no_manual_localization_button"] = True
            print("Client loaded", flush=True)

            page.locator("#togglecam").click()
            try:
                page.wait_for_function("""() => {
                    const v = document.querySelector('#cam');
                    return v.videoWidth > 0 && v.currentTime > 0.1;
                }""", timeout=25000)
                report["camera"] = page.locator("#cam").evaluate(
                    "v => ({width:v.videoWidth,height:v.videoHeight,time:v.currentTime})")
                page.locator("#cam").screenshot(path=str(output / "camera.png"))
                report["checks"]["camera_live"] = True
            except Exception as exc:
                report["checks"]["camera_live"] = False
                report["camera_error"] = page.locator("#camstat").inner_text() + ": " + str(exc)
            page.locator("#togglecam").click()
            print("Camera checked", flush=True)

            reset_started = True
            with page.expect_response(lambda r: r.url.endswith("/reset_map"), timeout=40000) as response:
                page.locator("#resetmapbtn").click()
            assert response.value.ok
            fresh = page.request.get(url + "/map").json()
            assert fresh["map_mode"] in ("starting_mapping", "updating")
            assert not fresh["localization"]["movement"]["active"]
            report["checks"]["new_map_skips_localization"] = True
            report["new_map_status"] = fresh["map_mode"]
            expect(page.locator('#mapphase')).to_have_text(
                re.compile('Starting map updates|Map updates active'), timeout=10000)
            expect(page.locator('#mapstat')).not_to_contain_text('localized')
            if not fresh['available']:
                page.wait_for_function("""() => {
                    const c = document.querySelector('#map');
                    const data = c.getContext('2d').getImageData(0,0,c.width,c.height).data;
                    return data.every((v,i) => i % 4 !== 3 || v === 0);
                }""", timeout=10000)
                report['checks']['new_map_clears_old_canvas'] = True
            page.screenshot(path=str(output / "new-map.png"), full_page=True)
            print("New map bypasses localization", flush=True)

            with page.expect_response(lambda r: r.url.endswith("/maps/load"), timeout=40000) as response:
                page.locator("#loadmapfile").set_input_files(bundle)
            assert response.value.ok, f"Map upload returned HTTP {response.value.status}"
            map_restored = True
            # Observe stationary matching rather than wait for a turn to start.
            if not allow_motion:
                assert page.request.post(url + '/estop', timeout=10000).ok
            deadline = time.monotonic() + 50
            while True:
                loaded = page.request.get(url + "/map").json()
                if loaded['map_mode'] in ('localizing','localization_error','localized','updating'):
                    break
                if time.monotonic() > deadline:
                    raise AssertionError(loaded['localization'])
                page.wait_for_timeout(500)
            assert loaded["map_mode"] in ("localizing", "localized", "updating"), loaded["localization"]
            report["checks"]["saved_map_starts_localization_automatically"] = True
            expect(page.locator("#mapdetail")).not_to_contain_text("Use Set Pose")
            report["loaded_status"] = loaded["localization"]
            page.screenshot(path=str(output / "loaded-map.png"), full_page=True)
            print("Saved map starts automatic localization", flush=True)

            if not allow_motion:
                with page.expect_response(lambda r: r.url.endswith("/estop")) as response:
                    page.locator("#stopbtn").click()
                assert response.value.ok
                stopped = page.request.get(url + "/map").json()
                assert not stopped["localization"]["movement"]["active"]
                report["checks"]["stop_cancels_movement"] = True
            observe_mapping(page, url, report, output, observe_seconds)
            assert not report["browser_errors"], report["browser_errors"]
        except Exception as exc:
            report['error'] = str(exc)
            raise
        finally:
            try:
                if reset_started and not map_restored:
                    # Preserve the prior saved map even when a UI assertion fails.
                    response = page.request.post(url + "/maps/load", data=bundle.read_bytes(),
                                                 headers={"Content-Type": "application/zip"}, timeout=30000)
                    report["fallback_restore_ok"] = response.ok
                    if not response.ok:
                        report['fallback_restore_error'] = response.text()
            except Exception as exc:
                report['fallback_restore_error'] = str(exc)
            try:
                page.request.post(url + "/estop", timeout=5000)
            except Exception as exc:
                report['stop_error'] = str(exc)
            finally:
                (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
                browser.close()
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--observe-seconds", type=float, default=90,
                        help="Stationary observation time after localization (default: 90)")
    parser.add_argument('--allow-localization-motion', action='store_true',
                        help='Allow the robot to execute its guarded small turns during localization')
    args = parser.parse_args()
    if not args.bundle.is_file():
        parser.error("A saved-map backup bundle is required")
    if not math.isfinite(args.observe_seconds) or args.observe_seconds < 0:
        parser.error("--observe-seconds must be finite and nonnegative")
    run(args.url.rstrip("/"), args.bundle, args.output, args.observe_seconds,
        args.allow_localization_motion)
