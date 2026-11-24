# Gamepad teleop (teleop_twist_joy) -> /cmd_vel -> the adapter's safety gate. Run it next to run_mapping.ps1 /
# run_localize.ps1. Hold LB and push the left stick (RB = faster). The pad must be attached to WSL (usbipd, see
# docs/ros2.md "Gamepad"). The robot must be armed in Manual (STATION) on the phone.
#   powershell -File ros2\teleop.ps1
#   powershell -File ros2\teleop.ps1 -WithAdapter -Url ws://192.168.0.209:8080/ws      nothing else running
param(
    [switch]$WithAdapter,
    [string]$Url = "ws://192.168.0.209:8080/ws",
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\launch.sh").Path
$sh = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
$a = @("teleop.launch.py")
if ($WithAdapter) { $a += @("adapter:=true", "url:=$Url") }
wsl -d $Distro -- bash $sh @a
