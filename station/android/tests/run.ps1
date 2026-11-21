# Desktop-JVM tests for the pure helpers (src\...\Lan.java, TiltMath.java, AppStop.java, HostPolicy.java, ...); no phone needed.
#   -Live   also scan this PC's 192.168.x /24 for the bridge with the app's own probe code (Finder.fetchStatus,
#           48 threads, same timeouts). Read-only: GET /api/status only.
param([switch]$Live)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$jdk = @('C:\Program Files\Microsoft\jdk-17.0.21.21-hotspot', 'C:\Program Files\Android\jdk') | Where-Object { Test-Path "$_\bin\javac.exe" } | Select-Object -First 1
if (-not $jdk) { throw 'No JDK found' }
$jar = "$env:LOCALAPPDATA\Android\Sdk\platforms\android-35\android.jar"
$out = Join-Path ([System.IO.Path]::GetTempPath()) ('sbr3-tests-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory $out | Out-Null
try {
    $src = "$root\src\com\sbr3\station"
    & "$jdk\bin\javac.exe" -encoding UTF-8 --release 8 -Xlint:-options -d $out "$src\Lan.java" "$src\TiltMath.java" "$src\AppStop.java" "$src\HostPolicy.java" "$src\NetPick.java" "$src\RateLimit.java" "$src\GestureRects.java" "$PSScriptRoot\LanTest.java" "$PSScriptRoot\AppStopTest.java" "$PSScriptRoot\HardeningTest.java"
    if ($LASTEXITCODE -ne 0) { throw "javac failed ($LASTEXITCODE)" }
    & "$jdk\bin\java.exe" -cp $out com.sbr3.station.LanTest
    $code = $LASTEXITCODE
    & "$jdk\bin\java.exe" -cp $out com.sbr3.station.AppStopTest
    if ($LASTEXITCODE -ne 0) { $code = $LASTEXITCODE }
    & "$jdk\bin\java.exe" -cp $out com.sbr3.station.HardeningTest
    if ($LASTEXITCODE -ne 0) { $code = $LASTEXITCODE }
    if ($Live -and $code -eq 0) {
        # android.jar stubs are enough: the probe code only uses java.net
        $all = @(Get-ChildItem $src -Filter *.java | ForEach-Object FullName) + "$PSScriptRoot\ScanSmoke.java"
        & "$jdk\bin\javac.exe" -nowarn -encoding UTF-8 --release 8 -Xlint:-options -XDsuppressNotes -cp $jar -d $out $all
        if ($LASTEXITCODE -ne 0) { throw "javac failed ($LASTEXITCODE)" }
        & "$jdk\bin\java.exe" -cp "$out;$jar" com.sbr3.station.ScanSmoke
        $code = $LASTEXITCODE
    }
} finally {
    Remove-Item $out -Recurse -Force -ErrorAction SilentlyContinue
}
exit $code
