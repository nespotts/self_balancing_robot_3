# Drive to a place saved on the phone (Map > Places), with Nav2. Needs run_localize.ps1 (or run_nav.ps1) running,
# and the robot armed in Manual (STATION). Ctrl+C cancels.
#   powershell -File ros2\go_to.ps1 -Place kitchen
#   powershell -File ros2\go_to.ps1 -X 1.5 -Y 0.4          a map point (m) instead
param(
    [string]$Place = "",
    [double]$X = [double]::NaN,
    [double]$Y = [double]::NaN,
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\node.sh").Path
$sh = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
if ($Place) {
    wsl -d $Distro -- bash $sh go_to --ros-args -p "place:=$Place" -p timeout:=600.0
} elseif (-not [double]::IsNaN($X)) {
    wsl -d $Distro -- bash $sh go_to --ros-args -p "x:=$X" -p "y:=$Y" -p timeout:=600.0
} else {
    Write-Output "usage: go_to.ps1 -Place <name>   or   go_to.ps1 -X <m> -Y <m>"
}
