# Builds Balance Bot (Gradle-free): aapt2 + javac + d8 + zipalign + apksigner -> build\BalanceBot.apk
#   -Release      not debuggable (no chrome://inspect); default is a debuggable build
#   -Keystore     signing key, default %USERPROFILE%\.android\sbr3-station.jks (created on first build; keep it:
#                 a new key means uninstall + reinstall on the phone). Never inside the repo.
#   -VersionCode  default: now as yyDDDHHmm;  -VersionName default: yyyy.MM.dd-HHmm
param([switch]$Release, [string]$Keystore = "$env:USERPROFILE\.android\sbr3-station.jks", [int]$VersionCode = 0, [string]$VersionName = '')
$ErrorActionPreference = 'Continue'
Set-Location $PSScriptRoot

$now = Get-Date
if ($VersionCode -le 0) { $VersionCode = [int]('{0:yy}{1:000}{0:HHmm}' -f $now, $now.DayOfYear) }
if (-not $VersionName) { $VersionName = $now.ToString('yyyy.MM.dd-HHmm', [Globalization.CultureInfo]::InvariantCulture) }
Write-Host "versionCode $VersionCode, versionName $VersionName, $(if ($Release) { 'release' } else { 'debug' })"

$sdk = "$env:LOCALAPPDATA\Android\Sdk"
$bt  = "$sdk\build-tools\35.0.0"
$jar = "$sdk\platforms\android-35\android.jar"
$jdk = @('C:\Program Files\Microsoft\jdk-17.0.21.21-hotspot', 'C:\Program Files\Android\jdk') | Where-Object { Test-Path "$_\bin\javac.exe" } | Select-Object -First 1
if (-not $jdk) { $jdk = (Get-ChildItem 'C:\Program Files\Android\jdk', 'C:\Program Files\Microsoft' -Directory -ErrorAction SilentlyContinue | Where-Object { Test-Path "$($_.FullName)\bin\javac.exe" } | Select-Object -First 1).FullName }
if (-not $jdk) { throw 'No JDK found' }
$env:JAVA_HOME = $jdk
$env:PATH = "$jdk\bin;$env:PATH"

function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }

if (Test-Path build) { Remove-Item build -Recurse -Force }
New-Item -ItemType Directory -Force build\tmp\gen, build\tmp\classes, build\tmp\dex | Out-Null

& "$bt\aapt2.exe" compile --dir res -o build\tmp\res.zip; Check 'aapt2 compile'
# (no -A: aapt2 on Windows stores asset subfolders with '\' -- assets are zipped below with '/' names)
$link = @('link', '-o', 'build\tmp\base.apk', '-I', $jar, '--manifest', 'AndroidManifest.xml',
    '--min-sdk-version', '26', '--target-sdk-version', '34', '--version-code', "$VersionCode", '--version-name', $VersionName,
    '--replace-version', '--java', 'build\tmp\gen')
if (-not $Release) { $link += '--debug-mode' }
& "$bt\aapt2.exe" @link 'build\tmp\res.zip'; Check 'aapt2 link'

$srcs = @(Get-ChildItem src, build\tmp\gen -Recurse -Filter *.java | ForEach-Object FullName)
& javac -nowarn -encoding UTF-8 -Xlint:-options --release 8 -classpath $jar -d build\tmp\classes $srcs; Check 'javac'
& "$jdk\bin\jar.exe" cf build\tmp\classes.jar -C build\tmp\classes .; Check 'jar'
$mode = if ($Release) { '--release' } else { '--debug' }
& "$bt\d8.bat" $mode --min-api 26 --lib $jar --output build\tmp\dex build\tmp\classes.jar; Check 'd8'

Copy-Item build\tmp\base.apk build\tmp\unsigned.apk
Add-Type -AssemblyName System.IO.Compression, System.IO.Compression.FileSystem
$zip = [System.IO.Compression.ZipFile]::Open("$PSScriptRoot\build\tmp\unsigned.apk", 'Update')
[System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip, "$PSScriptRoot\build\tmp\dex\classes.dex", 'classes.dex') | Out-Null
Get-ChildItem assets -Recurse -File | ForEach-Object {
    $rel = 'assets/' + $_.FullName.Substring("$PSScriptRoot\assets\".Length).Replace('\', '/')
    [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip, $_.FullName, $rel, [System.IO.Compression.CompressionLevel]::Optimal) | Out-Null
}
$zip.Dispose()

& "$bt\zipalign.exe" -f -p 4 build\tmp\unsigned.apk build\tmp\aligned.apk; Check 'zipalign'

if (-not (Test-Path $Keystore)) {
    New-Item -ItemType Directory -Force (Split-Path $Keystore) | Out-Null
    & keytool -genkeypair -keystore $Keystore -alias sbr3 -keyalg RSA -keysize 2048 -validity 10000 `
        -storepass sbr3station -keypass sbr3station -dname 'CN=Balance Bot' 2>$null; Check 'keytool'
    Write-Host "Created signing key $Keystore (back it up: a new key means uninstall + reinstall)"
}
& "$bt\apksigner.bat" sign --ks $Keystore --ks-key-alias sbr3 --ks-pass pass:sbr3station --key-pass pass:sbr3station `
    --min-sdk-version 26 --v2-signing-enabled true --out build\BalanceBot.apk build\tmp\aligned.apk; Check 'apksigner'
Remove-Item build\tmp -Recurse -Force
Remove-Item build\BalanceBot.apk.idsig -ErrorAction SilentlyContinue

$kb = [math]::Round((Get-Item build\BalanceBot.apk).Length / 1KB)
Write-Host "Built $PSScriptRoot\build\BalanceBot.apk ($kb KB, $VersionName / $VersionCode)"
