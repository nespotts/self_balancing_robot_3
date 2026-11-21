# Teensy serial console over Wi-Fi: runs tools/robot_console.py on the robot's Orange Pi.
#   tools\robot_console.ps1                     read-only, Ctrl+C to quit
#   tools\robot_console.ps1 --seconds 5
#   tools\robot_console.ps1 --release           make the ground-station bridge let go of the port
#   tools\robot_console.ps1 --get-params        also list the firmware's params (writes one request)
# Host: $env:ROBOT_HOST (default "robot", see ~/.ssh/config).
$robotHost = if ($env:ROBOT_HOST) { $env:ROBOT_HOST } else { 'robot' }
$py = Join-Path $PSScriptRoot 'robot_console.py'

# copy with LF endings (git may have checked it out with CRLF)
$tmp = Join-Path ([IO.Path]::GetTempPath()) 'robot_console.py'
[IO.File]::WriteAllText($tmp, ([IO.File]::ReadAllText($py) -replace "`r`n", "`n"))

& ssh -o BatchMode=yes -o ConnectTimeout=8 $robotHost 'mkdir -p sbr-tools'
if ($LASTEXITCODE -ne 0) { Write-Host "robot_console: cannot reach $robotHost over ssh"; exit 1 }
& scp -q -o BatchMode=yes $tmp "${robotHost}:sbr-tools/robot_console.py"
if ($LASTEXITCODE -ne 0) { Write-Host 'robot_console: copy to the Pi failed'; exit 1 }
Remove-Item $tmp -ErrorAction SilentlyContinue

# -t: Ctrl+C reaches the console on the Pi. Plain args only (PowerShell 5.1 mangles quotes).
$remote = 'python3 -u sbr-tools/robot_console.py ' + ($args -join ' ')
if ([Console]::IsInputRedirected) { & ssh $robotHost $remote } else { & ssh -t $robotHost $remote }
exit $LASTEXITCODE
