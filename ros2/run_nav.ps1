# Nav2 (+ slam_toolbox or a saved map, + RViz on WSLg). Ctrl+C stops everything.
#   powershell -File ros2\run_nav.ps1 -Sim                           sim in WSL; arm it in the web UI (http://localhost:8080)
#   powershell -File ros2\run_nav.ps1                                the robot, mapping while it drives
#   powershell -File ros2\run_nav.ps1 -Localization amcl -Map /home/sbr3/maps/room.yaml
# The robot must be armed and in Manual (STATION) on the phone: ROS never arms it. Goals: RViz "2D Goal Pose".
param(
    [string]$Url = "ws://192.168.0.209:8080/ws",
    [switch]$Sim,
    [switch]$NoRviz,
    [ValidateSet("slam", "amcl", "bridge")][string]$Localization = "slam",
    [string]$Map = "",
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\run.sh").Path
$run = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
$target = if ($Sim) { "sim" } else { $Url }
$launchArgs = @("rviz:=$(if ($NoRviz) { 'false' } else { 'true' })", "localization:=$Localization")
if ($Map) { $launchArgs += "map:=$Map" }
wsl -d $Distro -- bash $run nav $target @launchArgs
