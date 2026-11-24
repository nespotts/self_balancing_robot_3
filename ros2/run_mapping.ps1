# Map with slam_toolbox (+ RViz on WSLg). Ctrl+C stops everything.
#   powershell -File ros2\run_mapping.ps1 -Sim                       sim in WSL, drives a tour by itself
#   powershell -File ros2\run_mapping.ps1                            the robot (ws://192.168.0.209:8080/ws)
#   powershell -File ros2\run_mapping.ps1 -Url ws://10.0.0.5:8080/ws -NoRviz
# Save the map from another window:  wsl -d Ubuntu-24.04 -- bash -lc "ros2 run nav2_map_server map_saver_cli -f ~/maps/room"
param(
    [string]$Url = "ws://192.168.0.209:8080/ws",
    [switch]$Sim,
    [switch]$NoRviz,
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\run.sh").Path
$run = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
$target = if ($Sim) { "sim" } else { $Url }
$rviz = if ($NoRviz) { "rviz:=false" } else { "rviz:=true" }
wsl -d $Distro -- bash $run mapping $target $rviz
