# Portable package of the standalone app (nyan Real / Spatial Wall): collects the
# windeployqt run tree from build/ into a clean folder and zips it, so it can be
# copied to another PC and run without installing. Run .\build.ps1 first.
# (The OBS plugin has its own package.ps1; this is the closed standalone build.)
param([string]$Version)
$ErrorActionPreference = "Stop"
$SRC = $PSScriptRoot
$BUILD = Join-Path $SRC "build"
$exe = Join-Path $BUILD "spatial-wall.exe"
if (-not (Test-Path $exe)) {
  throw "build first (build\spatial-wall.exe missing). Run .\build.ps1"
}
if (-not $Version) {
  $cml = Get-Content (Join-Path $SRC 'CMakeLists.txt') -Raw
  if ($cml -match 'project\s*\([^)]*VERSION\s+(\d+\.\d+\.\d+)') {
    $Version = "$($Matches[1])-dev"
  } else {
    throw "could not parse VERSION from CMakeLists.txt"
  }
}

$DIST = Join-Path $SRC "dist"
New-Item -ItemType Directory -Force $DIST | Out-Null
# Stage and zip under TEMP (outside the Dropbox-synced repo): staging inside
# dist/ let Dropbox lock the files mid-zip ("file in use" on Compress-Archive).
# Only the finished zip is copied into dist/.
$STAGEBASE = Join-Path $env:TEMP "nyan-standalone-pkg"
$ROOT = Join-Path $STAGEBASE "nyan-real-spatial-wall"
Remove-Item $STAGEBASE -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $ROOT | Out-Null

# Runtime files staged next to the exe by build/windeployqt.
foreach ($f in @("spatial-wall.exe", "Qt6Core.dll", "Qt6Gui.dll",
                 "Qt6Widgets.dll", "Qt6Network.dll", "dxcompiler.dll",
                 "dxil.dll", "app_icon.png")) {
  $p = Join-Path $BUILD $f
  if (Test-Path $p) { Copy-Item $p $ROOT -Force } else { Write-Warning "missing: $f" }
}
# Qt plugin folders (platform, style, image formats, TLS, etc.).
foreach ($d in @("platforms", "styles", "imageformats", "tls",
                 "networkinformation", "generic")) {
  $p = Join-Path $BUILD $d
  if (Test-Path $p) { Copy-Item $p (Join-Path $ROOT $d) -Recurse -Force }
}
# data/ (locale .ini, remote.html, .effect/.hlsli shaders, app_icon).
Copy-Item (Join-Path $BUILD "data") (Join-Path $ROOT "data") -Recurse -Force

# Visual C++ runtime: copy app-local from System32 so the package is portable
# without an install (these CRT DLLs are redistributable; 2015-2022 are binary
# compatible). Fall back to bundling vc_redist.x64.exe if they are not found.
$crtCopied = 0
foreach ($c in @("msvcp140.dll", "msvcp140_1.dll", "msvcp140_2.dll",
                 "vcruntime140.dll", "vcruntime140_1.dll")) {
  $p = Join-Path $env:WINDIR "System32\$c"
  if (Test-Path $p) { Copy-Item $p $ROOT -Force; $crtCopied++ }
}
if ($crtCopied -eq 0) {
  $vcredist = Join-Path $BUILD "vc_redist.x64.exe"
  if (Test-Path $vcredist) { Copy-Item $vcredist $ROOT -Force }
  Write-Warning "VC++ runtime DLLs not found in System32; bundled vc_redist.x64.exe instead."
}

# Third-party licenses (Qt LGPL etc.).
$tpl = Join-Path $SRC "THIRD_PARTY_LICENSES"
if (Test-Path $tpl) { Copy-Item $tpl $ROOT -Force }

# ASCII-only so the script parses under Windows PowerShell 5.1 (which reads
# .ps1 in the system ANSI code page, mangling non-ASCII without a BOM).
$readme = @(
  "nyan Real / Spatial Wall - portable ($Version)",
  "",
  "Run spatial-wall.exe. No install needed; it is a tray-resident app.",
  "Connect a supported AR headset over USB and it is detected automatically.",
  "Quit from the tray icon menu.",
  "",
  "If it does not start, run the bundled vc_redist.x64.exe once to install the",
  "Microsoft Visual C++ runtime. (If no vc_redist is bundled, the runtime is",
  "already included app-local next to the exe.)"
) -join "`r`n"
Set-Content -Path (Join-Path $ROOT "README-portable.txt") -Value $readme -Encoding utf8

# Zip inside TEMP, then copy only the finished archive into dist/ (retry in
# case Dropbox briefly locks the destination while syncing).
$tmpzip = Join-Path $STAGEBASE "nyan-real-spatial-wall-$Version-portable-x64.zip"
Compress-Archive -Path $ROOT -DestinationPath $tmpzip -Force
$zip = Join-Path $DIST "nyan-real-spatial-wall-$Version-portable-x64.zip"
for ($i = 0; $i -lt 5; $i++) {
  try { Copy-Item $tmpzip $zip -Force; break }
  catch { if ($i -eq 4) { throw }; Start-Sleep -Milliseconds 500 }
}
Remove-Item $STAGEBASE -Recurse -Force -ErrorAction SilentlyContinue
Write-Host "`nportable zip -> $zip" -ForegroundColor Green
Get-ChildItem $DIST -File | Where-Object Name -like "nyan-real-spatial-wall-*" |
  Select-Object Name, Length
