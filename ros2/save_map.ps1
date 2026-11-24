# Save the live slam_toolbox map as ~/maps/<Name> in WSL: pose graph (.posegraph/.data) + grid (.yaml/.pgm) + .png.
# Run it in a second window while run_mapping.ps1 (or run_nav.ps1, or replay.ps1) runs.
#   powershell -File ros2\save_map.ps1 -Name room
# Then: powershell -File ros2\run_localize.ps1 -Map room
param(
    [Parameter(Mandatory = $true)][string]$Name,
    [string]$Distro = "Ubuntu-24.04"
)
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = (Resolve-Path "$here\scripts\save_map.sh").Path
$sh = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
wsl -d $Distro -- bash $sh $Name
