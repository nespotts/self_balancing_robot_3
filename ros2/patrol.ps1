# Patrol places saved on the phone with the Nav2 waypoint follower. Needs run_localize.ps1 (or run_nav.ps1) running,
# and the robot armed in Manual (STATION). Ctrl+C cancels.
#   powershell -File ros2\patrol.ps1 -Places "Kitchen,Couch,Home"                   once, 3 s at each place
#   powershell -File ros2\patrol.ps1 -Places "Kitchen,Couch" -Loops 0 -Dwell 10      until Ctrl+C
param(
    [Parameter(Mandatory = $true)][string]$Places,
    [int]$Loops = 1,
    [double]$Dwell = 3.0,
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\node.sh").Path
$sh = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
wsl -d $Distro -- bash $sh patrol --ros-args -p "places:=$Places" -p "loops:=$Loops" -p "dwell:=$Dwell"
