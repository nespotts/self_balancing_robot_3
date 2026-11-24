# Record this run (rosbag2): the adapter's topics + /map + /tf + /tf_static + Nav2 topics, into a dated folder in WSL.
# Start it in a second window while run_mapping.ps1 / run_nav.ps1 / run_localize.ps1 runs. Ctrl+C stops it.
#   powershell -File ros2\record_run.ps1                 ~/sbr3-bags/20261124_201500
#   powershell -File ros2\record_run.ps1 -Name kitchen   ~/sbr3-bags/20261124_201500_kitchen
#   powershell -File ros2\record_run.ps1 -Seconds 120    stops by itself
# Replay it offline (adapter off): powershell -File ros2\replay.ps1 -Bag <the printed path>
param(
    [string]$Name = "",
    [int]$Seconds = 0,
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\record.sh").Path
$sh = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
$dur = if ($Seconds -gt 0) { "$Seconds" } else { "" }
wsl -d $Distro -- bash $sh "$Name" $dur
