# Replay a recorded run offline (adapter off, clock from the bag) to tune slam_toolbox or AMCL. Ctrl+C stops.
#   powershell -File ros2\replay.ps1 -Bag /home/sbr3/sbr3-bags/20261124_201500              rebuild the map
#   powershell -File ros2\replay.ps1 -Bag <bag> -Mode amcl -Map room                         localize on ~/maps/room
#   powershell -File ros2\replay.ps1 -Bag <bag> -SlamParams /home/sbr3/try.yaml -Rate 2 -NoRviz
# Save the rebuilt map from a second window: powershell -File ros2\save_map.ps1 -Name room_offline
param(
    [Parameter(Mandatory = $true)][string]$Bag,
    [ValidateSet("slam", "amcl")][string]$Mode = "slam",
    [string]$Map = "",
    [double]$Rate = 1.0,
    [string]$SlamParams = "",
    [switch]$NoRviz,
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\run.sh").Path
$run = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
$a = @("bag:=$Bag", "mode:=$Mode", "rate:=$Rate", "rviz:=$(if ($NoRviz) { 'false' } else { 'true' })")
if ($Map) { if ($Map -notmatch "/") { $Map = "/home/sbr3/maps/$Map.yaml" }; $a += "map:=$Map" }
if ($SlamParams) { $a += "slam_params:=$SlamParams" }
wsl -d $Distro -- bash $run replay none @a
