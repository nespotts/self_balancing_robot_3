# One-time, idempotent: WSL distro Ubuntu-24.04 + ROS 2 Jazzy (desktop, slam_toolbox, Nav2) + the sbr3_ros package.
#   powershell -ExecutionPolicy Bypass -File ros2\setup_wsl.ps1
# No admin needed (WSL2 must already be enabled). ~9 GB on C: (the distro's ext4.vhdx).
param(
    [string]$Distro = "Ubuntu-24.04",
    [string]$InstallDir = "$env:LOCALAPPDATA\wsl\Ubuntu-24.04"
)
$ErrorActionPreference = "Stop"
$env:WSL_UTF8 = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
function To-WslPath([string]$p) { $f = (Resolve-Path $p).Path; return "/mnt/" + $f.Substring(0, 1).ToLower() + $f.Substring(2).Replace("\", "/") }

$have = (wsl -l -q) -split "`r?`n" | ForEach-Object { $_.Trim() } | Where-Object { $_ -eq $Distro }
if (-not $have) {
    Write-Host "Installing WSL distro $Distro ..."
    wsl --install -d $Distro --no-launch 2>&1 | Out-Null
    $have = (wsl -l -q) -split "`r?`n" | ForEach-Object { $_.Trim() } | Where-Object { $_ -eq $Distro }
    if (-not $have) {
        # WSL older than 2.4 cannot read the current distro list: import Canonical's official WSL image instead
        Write-Host "  'wsl --install' could not (old WSL); importing the official Ubuntu image"
        $info = Invoke-RestMethod https://raw.githubusercontent.com/microsoft/WSL/master/distributions/DistributionInfo.json
        $d = $info.ModernDistributions.Ubuntu | Where-Object { $_.Name -eq $Distro }
        if (-not $d) { throw "no $Distro in the WSL distribution list" }
        $dl = "$env:LOCALAPPDATA\wsl-dl"
        New-Item -ItemType Directory -Force $dl | Out-Null
        $img = Join-Path $dl (Split-Path -Leaf $d.Amd64Url.Url)
        if (-not (Test-Path $img) -or (Get-FileHash $img -Algorithm SHA256).Hash -ne $d.Amd64Url.Sha256.ToUpper()) {
            $ProgressPreference = "SilentlyContinue"
            Invoke-WebRequest $d.Amd64Url.Url -OutFile $img
        }
        if ((Get-FileHash $img -Algorithm SHA256).Hash -ne $d.Amd64Url.Sha256.ToUpper()) { throw "SHA256 mismatch: $img" }
        New-Item -ItemType Directory -Force $InstallDir | Out-Null
        wsl --import $Distro $InstallDir $img --version 2
    }
}
Write-Host "ROS 2 Jazzy + slam_toolbox + Nav2 (apt, as root in $Distro; a few minutes the first time) ..."
wsl -d $Distro -u root -- bash (To-WslPath "$here\scripts\setup_ros.sh")
if ($LASTEXITCODE -ne 0) { throw "setup_ros.sh failed" }
wsl --terminate $Distro | Out-Null          # wsl.conf default user takes effect on the next start
Write-Host "Building sbr3_ros ..."
wsl -d $Distro -- bash (To-WslPath "$here\scripts\build.sh")
Write-Host "Done. Try: powershell -File ros2\run_mapping.ps1 -Sim"
