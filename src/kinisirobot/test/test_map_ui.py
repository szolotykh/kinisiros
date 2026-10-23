"""Offline Playwright regressions: every HTTP request is intercepted locally."""

import json
from pathlib import Path

import pytest

try:
    import playwright.sync_api as playwright
except ImportError:  # The browser regression is optional on robot-only hosts.
    playwright = None

pytestmark = pytest.mark.skipif(
    playwright is None,
    reason='Playwright is not installed',
)
HTML = Path(__file__).resolve().parents[1] / 'web/index.html'


@pytest.fixture
def client_page():
    """Serve the real client against fake endpoints; never contact a robot."""
    with playwright.sync_playwright() as runtime:
        browser = runtime.chromium.launch(headless=True)
        page = browser.new_page()
        errors = []
        requests = []
        page.on('pageerror', lambda error: errors.append(str(error)))

        def respond(route):
            """Return deterministic browser fixtures for every intercepted request."""
            path = route.request.url.split('http://robot.test')[-1].split('?')[0]
            requests.append(path)
            if path == '/':
                route.fulfill(content_type='text/html', body=HTML.read_text(encoding='utf-8'))
                return
            payload = {'ok': True}
            if path == '/map':
                payload = {'available': False, 'pose': None, 'map_mode': 'localizing',
                           'localization': {'movement': {'active': False}}}
            elif path == '/robot/settings':
                payload = {'localization_radius_m': .3, 'last_pose': None}
            route.fulfill(content_type='application/json', body=json.dumps(payload))

        page.route('**/*', respond)
        page.goto('http://robot.test/', wait_until='domcontentloaded')
        playwright.expect(page.locator('#mapphase')).to_have_text('Locating robot')
        yield page, requests
        assert not errors
        browser.close()


def test_paused_localization_keeps_confidence_visible_without_resume(client_page):
    """A motion stop must not hide matching failures or require a resume button."""
    page, requests = client_page
    page.evaluate("""() => updateMapProgress({map_mode:'localizing', localization:{
      search_scope:'global', scan_match:0.5, scan_points:24,
      movement:{active:false, paused:true, can_resume:true, message:'Stopped: tab hidden'}
    }})""")
    playwright.expect(page.locator('#mapphase')).to_have_text('Locating robot')
    playwright.expect(page.locator('#localizationresume')).to_have_count(0)
    playwright.expect(page.locator('#mapdetail')).to_contain_text('50% (65% required)')
    playwright.expect(page.locator('#mapdetail')).to_contain_text('different headings')
    playwright.expect(page.locator('#mapdetail')).to_contain_text('continues automatically')
    assert '/localization/resume' not in requests
    assert '/maps/load' not in requests
    playwright.expect(page.locator('#savedposestat')).not_to_contain_text('Set Pose once')


def test_paused_localization_still_reports_a_failed_sensor(client_page):
    """Pausing movement must not hide the reason stationary matching cannot finish."""
    page, _ = client_page
    page.evaluate("""() => updateMapProgress({map_mode:'localizing', localization:{
      movement:{active:false, paused:true, can_resume:true},
      sensors:{lidar:{state:'stale'}, odometry:{state:'ready'}}
    }})""")
    playwright.expect(page.locator('#localizationmovestatus')).to_have_text(
        'LiDAR: stale. Robot remains stopped.')


def test_localized_navigation_is_separate_from_map_update_failure(client_page):
    """The UI must not present an optional mapping failure as lost localization."""
    page, _ = client_page
    page.evaluate("""() => updateMapProgress({map_mode:'localized', localization:{
      map_update_error:'SLAM verification failed', movement:{active:false}
    }})""")
    playwright.expect(page.locator('#mapphase')).to_have_text('Position confirmed')
    playwright.expect(page.locator('#mapdetail')).to_contain_text('Ready for navigation')
    playwright.expect(page.locator('#mapdetail')).to_contain_text('SLAM verification failed')
    playwright.expect(page.locator('#startmapping')).to_be_visible()


def test_background_tab_uses_temporary_session_stop(client_page):
    """Disconnect must not masquerade as pressing the operator's explicit Stop."""
    page, requests = client_page
    page.evaluate('() => beaconStop()')
    page.wait_for_timeout(100)
    assert '/session/leave' in requests
    assert '/estop' not in requests


def test_blur_and_repeated_leave_never_send_manual_stop(client_page):
    page, requests = client_page
    page.evaluate("""() => {
      window.dispatchEvent(new Event('blur'));
      window.dispatchEvent(new Event('pagehide'));
      window.dispatchEvent(new Event('beforeunload'));
    }""")
    page.wait_for_timeout(100)
    assert requests.count('/session/leave') >= 3
    assert '/cmd_vel' not in requests
    assert '/estop' not in requests


def test_paused_fresh_map_explains_retained_unsaved_work(client_page):
    page, _ = client_page
    page.evaluate("() => updateMapProgress({map_mode:'mapping_paused', localization:{}})")
    playwright.expect(page.locator('#mapphase')).to_have_text('Mapping paused safely')
    playwright.expect(page.locator('#mapdetail')).to_contain_text('unsaved map is retained')
    playwright.expect(page.locator('#mapdetail')).to_contain_text('still save')


def test_background_window_cannot_keep_motion_heartbeat_alive(client_page):
    page, requests = client_page
    page.evaluate('() => { document.hasFocus = () => false; }')
    requests.clear()
    page.evaluate('async () => { await sendHeartbeat(); }')
    assert '/heartbeat' not in requests
    page.evaluate('async () => { document.hasFocus = () => true; await sendHeartbeat(); }')
    assert '/heartbeat' in requests


def test_live_lidar_with_blind_sectors_reports_coverage_not_offline(client_page):
    page, _ = client_page
    page.evaluate("""() => updateMapProgress({map_mode:'localizing', localization:{
      movement:{active:false, paused:true},
      sensors:{lidar:{state:'ready', coverage:{clear:false,
        reason:'LiDAR coverage incomplete: 5 of 36 sectors unobserved'}}}
    }})""")
    playwright.expect(page.locator('#localizationmovestatus')).to_contain_text('5 of 36 sectors')
    playwright.expect(page.locator('#localizationmovestatus')).not_to_contain_text('offline')


def test_hung_poll_reports_disconnect_and_can_recover(client_page):
    """An unresponsive connection must stop displaying stale localization progress."""
    page, _ = client_page
    page.evaluate("""() => {
      window.originalFetch = window.fetch;
      window.fetch = (url, options) => url.startsWith('/map')
        ? new Promise((resolve, reject) => options.signal.addEventListener('abort',
            () => reject(new DOMException('Aborted', 'AbortError'))))
        : originalFetch(url, options);
      void refreshMap();
    }""")
    playwright.expect(page.locator('#mapphase')).to_have_text('Robot disconnected', timeout=7000)
    playwright.expect(page.locator('#mode-goal')).to_be_disabled()
    page.evaluate('async () => { window.fetch = originalFetch; await refreshMap(); }')
    playwright.expect(page.locator('#mapphase')).to_have_text('Locating robot')


def test_old_response_cannot_overwrite_new_status(client_page):
    """Even a fetch implementation that ignores abort cannot regress the displayed state."""
    page, _ = client_page
    page.evaluate("""async () => {
      const previous = window.fetch;
      const responses = [];
      window.fetch = (url, options) => url.startsWith('/map')
        ? new Promise(resolve => responses.push(resolve)) : previous(url, options);
      const oldPoll = refreshMap();
      const newPoll = refreshMap();
      responses[1]({ok:true, json:async () => ({available:false, pose:null,
        map_mode:'waiting_mapping', localization:{}})});
      await newPoll;
      responses[0]({ok:true, json:async () => ({available:false, pose:null,
        map_mode:'localizing', localization:{}})});
      await oldPoll;
      window.fetch = previous;
    }""")
    playwright.expect(page.locator('#mapphase')).to_have_text('Waiting for mapping data')


def test_completed_localization_restores_navigation_hint(client_page):
    """A finished map load must not keep warning that automatic turns are active."""
    page, _ = client_page
    page.evaluate("""() => drawMap({available:false, pose:null,
      map_mode:'updating', localization:{}})""")
    playwright.expect(page.locator('#mode-goal')).to_be_disabled()
    playwright.expect(page.locator('#navhint')).to_have_text('Drive mode: use keys to teleop.')


def test_goal_can_be_selected_while_localizing_without_stopping_it(client_page):
    """Selecting a destination must not send a manual zero that cancels turns."""
    page, requests = client_page
    page.evaluate("""() => {
      drawMap({available:true, width:2, height:2, resolution:1,
        origin:{x:0,y:0,yaw:0}, data:[0,0,0,0],
        pose:null, map_mode:'localizing', localization:{}});
    }""")
    page.locator('#mode-goal').click()
    assert '/cmd_vel' not in requests
    page.route('**/nav_goal', lambda route: route.fulfill(
        content_type='application/json', body='{"ok":true,"state":"waiting_localization"}'))
    page.evaluate("""async () => {
      dragStart = [1, 2]; pendingPose = {x:1, y:2, yaw:0};
      await finishDrag();
      updateNavStatus({state:'waiting_localization', goal:{x:1,y:2,yaw:0}});
    }""")
    playwright.expect(page.locator('#navhint')).to_contain_text('waiting for localization')
    playwright.expect(page.locator('#navstate')).to_have_text('Waiting for localization')
