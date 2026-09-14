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
      * web UI      - robot_client/web_bridge.py on port 8080 (http://rospi.local:8080/).
      * nav+mapping - navigation.launch.py (Nav2). With -Mode update-map (default) it
                      also runs slam_toolbox so you NAVIGATE and MAP at the same time.

    All processes are launched detached (setsid nohup) so they survive the SSH session.
    The script is idempotent: components already running are left alone (use -Action
    restart to force a clean relaunch). Note: nothing survives a Pi REBOOT - just
    re-run this script.

    On start/restart the script also DEPLOYS the latest robot_client code (web_bridge.py,
    index.html and cam_grab.py) to the Pi via scp, then force-relaunches
    web_bridge so the fresh Python + HTML take effect immediately. Use -SkipDeploy to skip
    the copy and just (re)start whatever is already on the Pi.

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

    # Skip copying the local robot_client code to the Pi before (re)starting.
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
    # web_bridge traps SIGTERM (via rclpy) and tears down its ROS context but
    # leaves the HTTP server thread running -> a half-dead "zombie" that keeps
    # port 8080 bound while every /cmd_vel & /estop throws (no drive). Detect
    # that (port up but /web_bridge node gone) and relaunch a clean one.
    # WEBDEPLOY=1 means fresh code was just scp'd, so force a relaunch even when
    # the node is healthy - otherwise the old code stays live until next restart.
    if is_listening "$WEBPORT" && node_up "/web_bridge"; then
        if [ "$WEBDEPLOY" = "1" ]; then
            echo "[web UI]    code redeployed - relaunching web_bridge (:$WEBPORT)"
            pkill -9 -f "web_bridge.py" 2>/dev/null
            sleep 1
        else
            echo "[web UI]    already up (:$WEBPORT)"
            return
        fi
    elif is_listening "$WEBPORT"; then
        echo "[web UI]    port :$WEBPORT bound but /web_bridge node is dead - relaunching"
        pkill -9 -f "web_bridge.py" 2>/dev/null
        sleep 1
    else
        echo "[web UI]    starting web_bridge.py (:$WEBPORT)"
    fi
    ( cd "$WS/robot_client" && setsid nohup python3 web_bridge.py --port "$WEBPORT" \
        >"$LOG/web_bridge.log" 2>&1 </dev/null & ) </dev/null >/dev/null 2>&1
}

start_nav() {
    # MODE update-map -> slam:=true (Nav2 + mapping); localization -> slam:=false (Nav2 + AMCL)
    if node_up "/bt_navigator"; then echo "[nav]       already running"; return; fi
    local slam args
    if [ "$MODE" = "update-map" ]; then slam=true; else slam=false; fi
    args="slam:=$slam"
    if [ "$slam" = "true" ] && [ -n "$SLAM_MAP" ]; then args="$args slam_map:=$SLAM_MAP"; fi
    echo "[nav]       waiting for base, then launching navigation.launch.py $args"
    wait_node "/kinisi_controller" || echo "[nav]       WARNING: base not detected, launching anyway"
    ( cd "$WS" && setsid nohup ros2 launch kinisirobot navigation.launch.py $args \
        >"$LOG/nav_mapping.log" 2>&1 </dev/null & ) </dev/null >/dev/null 2>&1
}

start_mapping_only() {
    if node_up "/slam_toolbox"; then echo "[mapping]   already running"; return; fi
    echo "[mapping]   waiting for base, then launching slam_toolbox (online async)"
    wait_node "/kinisi_controller" || echo "[mapping]   WARNING: base not detected, launching anyway"
    ( cd "$WS" && setsid nohup ros2 launch slam_toolbox online_async_launch.py \
        slam_params_file:="$WS/src/kinisirobot/config/mapper_params_online_async.yaml" \
        use_sim_time:=false >"$LOG/slam.log" 2>&1 </dev/null & ) </dev/null >/dev/null 2>&1
}

stop_all() {
    echo "Stopping omni stack ..."
    for pat in \
        "navigation.launch.py" "online_async_launch" "async_slam_toolbox_node" \
        "controller_server" "planner_server" "behavior_server" "bt_navigator" \
        "map_server" "amcl" "lifecycle_manager" \
        "rosbridge_websocket" "web_bridge.py" \
        "rsp.launch.py" "kinisi_controller" "rplidar_composition" \
        "robot_state_publisher" "static_transform_publisher"; do
        pkill -f "$pat" 2>/dev/null
    done
    sleep 2
    # web_bridge ignores SIGTERM cleanly (rclpy only tears down the ROS context
    # and leaves the HTTP server bound to :8080, a zombie). Escalate to SIGKILL
    # so the port is truly freed and the next start relaunches a clean node.
    pkill -9 -f "web_bridge.py" 2>/dev/null
    echo "Stopped."
}

status() {
    local h; h="$(hostname).local"
    echo "=== OMNI stack status ($(hostname)) ==="
    if is_listening "$WEBPORT"; then echo "  web UI    : UP    http://$h:$WEBPORT/"; else echo "  web UI    : DOWN"; fi
    if is_listening 9090;      then echo "  rosbridge : UP    ws://$h:9090"; else echo "  rosbridge : DOWN"; fi
    if node_up "/kinisi_controller"; then echo "  base      : UP    (controller + lidar + TF)"; else echo "  base      : DOWN"; fi
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

# Deploy the local robot_client code on start/restart (unless -SkipDeploy).
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
    param([string[]]$Sources, [string]$Dest, [string]$Label)
    $scpBase = @(
        '-i', $SshKey,
        '-o', 'IdentitiesOnly=yes',
        '-o', 'ConnectTimeout=15',
        '-o', 'StrictHostKeyChecking=accept-new'
    )
    for ($try = 1; $try -le 3; $try++) {
        & scp @scpBase @Sources $Dest
        if ($LASTEXITCODE -eq 0) { return $true }
        Write-Host "  scp $Label attempt $try/3 failed (exit $LASTEXITCODE) - retrying in 5s ..." -ForegroundColor Yellow
        Start-Sleep -Seconds 5
    }
    return $false
}

if ($doDeploy) {
    $wsRemote = 'development/kinisiros'
    $allOk = $true

    # 1) Web UI + capture script -> robot_client/ (README.md is not deployed).
    $clientDir = Join-Path $PSScriptRoot 'robot_client'
    $clientFiles = Get-ChildItem -Path "$clientDir\*" -File -Include '*.py', '*.sh', '*.html'
    if ($clientFiles) {
        Write-Host "Deploying $($clientFiles.Count) robot_client file(s) to $RobotHost ..." -ForegroundColor Cyan
        if (-not (Invoke-ScpWithRetry -Sources $clientFiles.FullName -Dest "${User}@${RobotHost}:$wsRemote/robot_client/" -Label 'robot_client')) { $allOk = $false }
    }

    # 2) Workspace source files we edit (ROS node + Nav2 config). The install is
    #    symlinked to these sources, so copying them over updates the live code -
    #    but the base + Nav2 nodes only pick it up on a RESTART (use -Action restart).
    $srcMap = @{
        'src\roskinisi\roskinisi\kinisi_controller.py' = "$wsRemote/src/roskinisi/roskinisi/kinisi_controller.py"
        'src\kinisirobot\config\nav2_params.yaml'      = "$wsRemote/src/kinisirobot/config/nav2_params.yaml"
    }
    foreach ($rel in $srcMap.Keys) {
        $local = Join-Path $PSScriptRoot $rel
        if (Test-Path $local) {
            Write-Host "Deploying $rel ..." -ForegroundColor Cyan
            if (-not (Invoke-ScpWithRetry -Sources @($local) -Dest "${User}@${RobotHost}:$($srcMap[$rel])" -Label $rel)) { $allOk = $false }
        }
    }

    if ($allOk) {
        Write-Host "Deploy: OK. Source/config changes need -Action restart to take effect." -ForegroundColor Green
    }
    else {
        Write-Host "Deploy: some files FAILED after 3 attempts - continuing with the code already on the Pi." -ForegroundColor Red
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
