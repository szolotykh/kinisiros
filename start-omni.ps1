<#
.SYNOPSIS
    Start (or stop / check) the full Kinisi OMNI robot stack on the Raspberry Pi
    from this Windows PC over SSH.

.DESCRIPTION
    Brings up, on the Pi (rospi.local, ROS 2 Humble, ws ~/development/kinisiros):
      * base        - rsp.launch.py (kinisi_controller + rplidar + robot_state_publisher
                      + base_link->laser_frame TF): publishes /odom + odom->base_link,
                      subscribes /cmd_vel.
      * rosbridge   - rosbridge_server websocket on port 9090 (for roslibjs / Foxglove).
      * web UI      - kinisirobot/kinisi_web_bridge on port 8080 (http://rospi.local:8080/).
      * nav+mapping - navigation.launch.py (Nav2). With -Mode update-map (default) it
                      also runs slam_toolbox so you NAVIGATE and MAP at the same time.

    All processes are launched detached (setsid nohup) so they survive the SSH session.
    The script is idempotent: components already running are left alone (use -Action
    restart to force a clean relaunch). Note: nothing survives a Pi REBOOT - just
    re-run this script.

    On start/restart the script also deploys the ROS package sources, rebuilds
    the affected packages, and relaunches the web bridge. Use -SkipDeploy to
    run the packages already installed on the Pi.

.PARAMETER Action
    start (default) | stop | status | restart

.PARAMETER Mode
    update-map (default) - Nav2 + slam_toolbox mapping together (navigate while mapping).
    localization         - Nav2 + AMCL on the saved static map (map does not change).
    mapping              - slam_toolbox only (build a map, no Nav2).
    none                 - base + rosbridge + web UI only (no SLAM, no Nav2).

.PARAMETER SlamMap
    (update-map only) Basename of a serialized map to continue extending
    (expects <name>.posegraph + <name>.data). Empty = start a fresh map.

.EXAMPLE
    .\start-omni.ps1
    # base + rosbridge + web UI + Nav2 + mapping (navigate and map at once)

.EXAMPLE
    .\start-omni.ps1 -Action status
    .\start-omni.ps1 -Action restart
    .\start-omni.ps1 -Action stop
    .\start-omni.ps1 -Mode none          # just the robot + UI, no autonomy
    .\start-omni.ps1 -Mode localization  # Nav2 on the saved map

.EXAMPLE
    .\start-omni.ps1 -Action restart              # redeploy code + clean relaunch
    .\start-omni.ps1 -Action restart -SkipDeploy  # relaunch without copying code
    .\start-omni.ps1 -RobotHost 192.168.68.91     # if mDNS (rospi.local) won't resolve
#>
[CmdletBinding()]
param(
    [ValidateSet('start', 'stop', 'status', 'restart')]
    [string]$Action = 'start',

    [ValidateSet('update-map', 'localization', 'mapping', 'none')]
    [string]$Mode = 'update-map',

    [int]$WebPort = 8080,
    [string]$RobotHost = 'rospi.local',
    [string]$User = 'szolotykh',
    [string]$SshKey = "$HOME\.ssh\pi_kinisi",
    [string]$SlamMap = '',

    # Skip copying and rebuilding the local ROS packages before (re)starting.
    [switch]$SkipDeploy
)

$ErrorActionPreference = 'Stop'

# --- remote bash payload (single-quoted here-string: no PowerShell expansion) ---
$remoteScript = @'
#!/usr/bin/env bash
set -o pipefail
ACTION="${1:-start}"        # start | stop | status
MODE="${2:-update-map}"     # none | update-map | localization | mapping
WEBPORT="${3:-8080}"
RESTART="${4:-0}"
SLAM_MAP="${5:-}"
WEBDEPLOY="${6:-0}"     # 1 = code was just redeployed, force web_bridge relaunch

WS="$HOME/development/kinisiros"
LOG="$HOME"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Pre-start the ROS 2 daemon fully detached. Otherwise the first ros2 CLI call
# below (node list / launch) spawns a daemon that INHERITS this SSH session's
# stdout pipe and never releases it, so the ssh command - and start-omni.ps1 -
# appears to "hang" / never exit even though all the work is already done.
setsid ros2 daemon start </dev/null >/dev/null 2>&1 || true

is_listening() { ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE ":$1$"; }
node_up()      { timeout 8 ros2 node list 2>/dev/null | grep -qx "$1"; }
proc_up()      { pgrep -f "$1" >/dev/null 2>&1; }

wait_node() { local n="$1" i; for i in $(seq 1 30); do node_up "$n" && return 0; sleep 1; done; return 1; }

start_base() {
    if proc_up "rsp.launch.py" || node_up "/kinisi_controller"; then
        echo "[base]      already running"
    else
        echo "[base]      starting rsp.launch.py (controller + lidar + TF)"
        ( cd "$WS" && setsid nohup ros2 launch kinisirobot rsp.launch.py \
            >"$LOG/omni_robot.log" 2>&1 </dev/null & ) </dev/null >/dev/null 2>&1
    fi
}

start_rosbridge() {
    if is_listening 9090; then
        echo "[rosbridge] already up (:9090)"
    else
        echo "[rosbridge] starting websocket (:9090)"
        ( cd "$WS" && setsid nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
            >"$LOG/rosbridge.log" 2>&1 </dev/null & ) </dev/null >/dev/null 2>&1
    fi
}

start_web() {
    # A live web UI means BOTH the port is bound AND the rclpy node is alive.
    # Older bridge versions could leave HTTP alive after losing the ROS node.
    # Detect that legacy state. Healthy versions shut down owned map backends
    # on SIGTERM before releasing the HTTP port.
    # WEBDEPLOY=1 means fresh code was just scp'd, so force a relaunch even when
    # the node is healthy - otherwise the old code stays live until next restart.
    if is_listening "$WEBPORT" && node_up "/web_bridge"; then
        if [ "$WEBDEPLOY" = "1" ]; then
            echo "[web UI]    code redeployed - relaunching web_bridge (:$WEBPORT)"
            pkill -TERM -f "kinisi_web_bridge" 2>/dev/null
            for attempt in $(seq 1 20); do
                is_listening "$WEBPORT" || break
                sleep 1
            done
            if is_listening "$WEBPORT"; then
                echo "[web UI]    shutdown incomplete; refusing to orphan map backends"
                return 1
            fi
        else
            echo "[web UI]    already up (:$WEBPORT)"
            return
        fi
    elif is_listening "$WEBPORT"; then
        echo "[web UI]    port :$WEBPORT bound but /web_bridge node is dead - relaunching"
        pkill -9 -f "kinisi_web_bridge" 2>/dev/null
        sleep 1
    else
        echo "[web UI]    starting kinisi_web_bridge (:$WEBPORT)"
    fi
    local web_args=()
    if [ "$MODE" = "update-map" ] || [ "$MODE" = "mapping" ]; then
        if [ -n "$SLAM_MAP" ]; then
            web_args+=(--localize-map "$SLAM_MAP")
        else
            web_args+=(--start-mapping)
        fi
    fi
    ( cd "$WS" && setsid nohup ros2 run kinisirobot kinisi_web_bridge --port "$WEBPORT" "${web_args[@]}" \
        >"$LOG/web_bridge.log" 2>&1 </dev/null & ) </dev/null >/dev/null 2>&1
}

start_nav() {
    # In update-map mode the bridge owns SLAM; this launch starts only Nav2.
    # Standalone localization mode keeps its existing externally managed backend.
    if node_up "/bt_navigator"; then echo "[nav]       already running"; return; fi
    local slam args
    if [ "$MODE" = "update-map" ]; then slam=true; else slam=false; fi
    args="slam:=$slam"
    if [ "$slam" = "true" ]; then args="$args backend:=false"; fi
    echo "[nav]       waiting for base, then launching navigation.launch.py $args"
    wait_node "/kinisi_controller" || echo "[nav]       WARNING: base not detected, launching anyway"
    ( cd "$WS" && setsid nohup ros2 launch kinisirobot navigation.launch.py $args \
        >"$LOG/nav_mapping.log" 2>&1 </dev/null & ) </dev/null >/dev/null 2>&1
}

start_mapping_only() {
    # The web bridge owns mapping in this mode; do not launch a second provider.
    echo "[mapping]   managed by the web bridge"
}

stop_all() {
    echo "Stopping omni stack ..."
    for pat in \
        "navigation.launch.py" "online_async_launch" "async_slam_toolbox_node" \
        "controller_server" "planner_server" "behavior_server" "bt_navigator" \
        "map_server" "amcl" "lifecycle_manager" \
        "rosbridge_websocket" "kinisi_web_bridge" \
        "rsp.launch.py" "kinisi_controller" "kinisi_lidar_supervisor" "rplidar_composition" \
        "robot_state_publisher" "static_transform_publisher"; do
        pkill -f "$pat" 2>/dev/null
    done
    sleep 2
    # Clean shutdown is expected; this final kill handles legacy bridge versions.
    pkill -9 -f "kinisi_web_bridge" 2>/dev/null
    echo "Stopped."
}

status() {
    local h; h="$(hostname).local"
    echo "=== OMNI stack status ($(hostname)) ==="
    if is_listening "$WEBPORT"; then echo "  web UI    : UP    http://$h:$WEBPORT/"; else echo "  web UI    : DOWN"; fi
    if is_listening 9090;      then echo "  rosbridge : UP    ws://$h:9090"; else echo "  rosbridge : DOWN"; fi
    if node_up "/kinisi_controller"; then echo "  base      : UP    (controller + lidar + TF)"; else echo "  base      : DOWN"; fi
    if node_up "/lidar_supervisor"; then echo "  lidar     : SUPERVISED"; else echo "  lidar     : NO SUPERVISOR"; fi
    if node_up "/slam_toolbox";      then echo "  mapping   : UP    (slam_toolbox)"; else echo "  mapping   : DOWN"; fi
    if node_up "/bt_navigator";      then echo "  navigation: UP    (nav2)"; else echo "  navigation: DOWN"; fi
}

case "$ACTION" in
    stop)   stop_all; echo; status ;;
    status) status ;;
    start)
        if [ "$RESTART" = "1" ]; then stop_all; echo; fi
        start_base
        start_rosbridge
        start_web
        if [ "$MODE" = "mapping" ]; then
            start_mapping_only
        elif [ "$MODE" != "none" ]; then
            start_nav
        else
            echo "[nav]       skipped (Mode=none)"
        fi
        echo "Waiting for nodes to come up ..."
        sleep 12
        echo
        status
        ;;
    *) echo "unknown action: $ACTION"; exit 1 ;;
esac
'@

# --- map PowerShell action -> remote (start/stop/status) + restart flag ---
$restartFlag = if ($Action -eq 'restart') { '1' } else { '0' }
$remoteAction = if ($Action -eq 'restart') { 'start' } else { $Action }

# Deploy and rebuild local ROS package sources on start/restart (unless -SkipDeploy).
$doDeploy = ($Action -in @('start', 'restart')) -and (-not $SkipDeploy)
$webDeployFlag = if ($doDeploy) { '1' } else { '0' }

$remoteArgs = "$remoteAction $Mode $WebPort $restartFlag '$SlamMap' $webDeployFlag"

$sshArgs = @(
    '-i', $SshKey,
    '-o', 'IdentitiesOnly=yes',
    '-o', 'ConnectTimeout=15',
    '-o', 'StrictHostKeyChecking=accept-new',
    "$User@$RobotHost"
)

Write-Host "== Kinisi OMNI control ==  host=$RobotHost  action=$Action  mode=$Mode" -ForegroundColor Cyan

# --- deploy latest code to the Pi (scp) ---
# Shared scp-with-retry helper (mDNS to rospi.local can be flaky).
function Invoke-ScpWithRetry {
    param([string[]]$Sources, [string]$Dest, [string]$Label, [switch]$Recurse)
    $scpBase = @(
        '-i', $SshKey,
        '-o', 'IdentitiesOnly=yes',
        '-o', 'ConnectTimeout=15',
        '-o', 'StrictHostKeyChecking=accept-new'
    )
    for ($try = 1; $try -le 3; $try++) {
        $scpArgs = if ($Recurse) { @('-r') + $scpBase } else { $scpBase }
        & scp @scpArgs @Sources $Dest
        if ($LASTEXITCODE -eq 0) { return $true }
        Write-Host "  scp $Label attempt $try/3 failed (exit $LASTEXITCODE) - retrying in 5s ..." -ForegroundColor Yellow
        Start-Sleep -Seconds 5
    }
    return $false
}

if ($doDeploy) {
    $wsRemote = 'development/kinisiros'
    $allOk = $true

    $packageSources = @(
        (Join-Path $PSScriptRoot 'src\kinisirobot'),
        (Join-Path $PSScriptRoot 'src\roskinisi')
    )
    Write-Host "Deploying kinisirobot and roskinisi package sources ..." -ForegroundColor Cyan
    if (-not (Invoke-ScpWithRetry -Sources $packageSources -Dest "${User}@${RobotHost}:$wsRemote/src/" -Label 'ROS packages' -Recurse)) {
        $allOk = $false
    }

    if ($allOk) {
        $buildCommand = "cd $wsRemote && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select roskinisi kinisirobot"
        Write-Host "Building deployed ROS packages ..." -ForegroundColor Cyan
        ssh @sshArgs $buildCommand
        if ($LASTEXITCODE -ne 0) { $allOk = $false }
    }

    if ($allOk) {
        Write-Host "Deploy/build: OK. Package changes are ready for restart." -ForegroundColor Green
    }
    else {
        throw 'Deploy/build failed. Startup aborted; inspect the deployment before restarting the robot.'
    }
}

# Base64-encode the (LF-normalized) payload so Windows CRLF line endings and
# shell quoting can't corrupt it in transit; decode + run it on the Pi.
$lf = $remoteScript -replace "`r`n", "`n"
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($lf))
$remoteCmd = "echo $b64 | base64 -d | bash -s -- $remoteArgs"

# rospi.local is mDNS and can be flaky to resolve from Windows - retry a few times.
$maxTries = 5
$ok = $false
for ($try = 1; $try -le $maxTries; $try++) {
    ssh @sshArgs $remoteCmd
    if ($LASTEXITCODE -eq 0) { $ok = $true; break }
    Write-Host "SSH attempt $try/$maxTries failed (exit $LASTEXITCODE) - retrying in 5s ..." -ForegroundColor Yellow
    Start-Sleep -Seconds 5
}

if (-not $ok) {
    Write-Host "Could not reach $RobotHost after $maxTries attempts." -ForegroundColor Red
    Write-Host "The Pi may be off / off Wi-Fi, or mDNS is not resolving. Try:" -ForegroundColor Red
    Write-Host "  .\start-omni.ps1 -RobotHost <pi-ip-address>" -ForegroundColor Red
    exit 1
}

if ($Action -in @('start', 'restart')) {
    Write-Host ""
    Write-Host "Open the UI:  http://$RobotHost`:$WebPort/" -ForegroundColor Green
    Write-Host "rosbridge  :  ws://$RobotHost`:9090" -ForegroundColor Green
    if ($Mode -eq 'update-map') {
        Write-Host "Nav+mapping running: drive to build the map, send goals via the UI 'Set Goal'." -ForegroundColor Green
    } elseif ($Mode -eq 'localization') {
        Write-Host "Nav2 (localization): seed the pose with the UI 'Set Pose' before sending goals." -ForegroundColor Green
    }
}
