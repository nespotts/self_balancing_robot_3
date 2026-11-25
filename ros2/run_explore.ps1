# Explore (#107): slam_toolbox maps while a frontier explorer drives Nav2 until no reachable frontier is left.
# Ctrl+C stops everything. No RViz by default: watch the map and the status on the station's Map page.
#   powershell -File ros2\run_explore.ps1                     the robot; start / stop with "Explore (ROS)" on the Map page
#   powershell -File ros2\run_explore.ps1 -Start              the robot; start exploring as soon as it is armed in Manual
#   powershell -File ros2\run_explore.ps1 -Sim -Start         sim in WSL (web UI http://localhost:8080), armed for you
#   powershell -File ros2\run_explore.ps1 -Sim -Maze -Start   the sim's maze room
#   powershell -File ros2\run_explore.ps1 -Rviz               with RViz
# The robot must be armed and in Manual (STATION) on the phone: ROS never arms it. The map is saved by itself when
# exploring ends (~/maps/explore-<date>-<time> in WSL); or save it any time: powershell -File ros2\save_map.ps1 -Name x
param(
    [string]$Url = "ws://192.168.0.209:8080/ws",
    [switch]$Sim,
    [switch]$Maze,
    [switch]$Start,
    [switch]$Rviz,
    [switch]$NoRviz,          # the default; kept so the other scripts' habit works
    [switch]$ReturnHome,
    [switch]$NoKeepout,       # no station zones as keep-out (on with -Maze: the sim's maze walls are real walls)
    [double]$MaxMinutes = 0,
    [string]$MapDir = "~/maps",
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\run.sh").Path
$run = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
$target = if ($Sim) { "sim" } else { $Url }
if ($Sim) {
    $env:SBR3_SIM_ARGS = if ($Maze) { "--maze" } else { "" }
    $env:SBR3_SIM_ARM = if ($Start) { "1" } else { "" }       # sim only: arm + Manual, as you would in the web UI
    $env:WSLENV = "SBR3_SIM_ARGS/u:SBR3_SIM_ARM/u"
}
$launchArgs = @("rviz:=$(if ($Rviz -and -not $NoRviz) { 'true' } else { 'false' })",
                "autostart:=$(if ($Start) { 'true' } else { 'false' })",
                "return_home:=$(if ($ReturnHome) { 'true' } else { 'false' })",
                "max_duration_s:=$([string]($MaxMinutes * 60))",
                "keepout:=$(if ($NoKeepout -or $Maze) { 'false' } else { 'true' })",
                "map_dir:=$MapDir")
wsl -d $Distro -- bash $run explore $target @launchArgs
