# Localize on a saved map (save_map.ps1) + Nav2 + RViz. Ctrl+C stops everything.
#   powershell -File ros2\run_localize.ps1 -Map room                         the robot; it finds itself on the map
#   powershell -File ros2\run_localize.ps1 -Map room -Localization amcl      AMCL instead of slam_toolbox localization
#   powershell -File ros2\run_localize.ps1 -Map room -InitialPose "0,0,0"    start pose given (x m, y m, yaw rad)
#   powershell -File ros2\run_localize.ps1 -Map simroom -Sim -SimStart "2600,-200,135"     sim, started elsewhere
# Then Arm & Balance + Manual on the phone, and goals in RViz, go_to.ps1 or patrol.ps1.
param(
    [Parameter(Mandatory = $true)][string]$Map,
    [ValidateSet("slam", "amcl")][string]$Localization = "slam",
    [string]$InitialPose = "",
    [string]$Url = "ws://192.168.0.209:8080/ws",
    [switch]$Sim,
    [string]$SimStart = "",
    [switch]$NoRviz,
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\run.sh").Path
$run = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
$target = if ($Sim) { "sim" } else { $Url }
if ($SimStart) { $env:SBR3_SIM_ARGS = "--start $SimStart"; $env:WSLENV = "SBR3_SIM_ARGS/u" }
$a = @("map:=$Map", "localization:=$Localization", "rviz:=$(if ($NoRviz) { 'false' } else { 'true' })")
if ($InitialPose) { $a += "initial_pose:=$InitialPose" }
wsl -d $Distro -- bash $run localize $target @a
