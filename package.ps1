# Stage the built plugin into the OBS plugin layout, then produce a portable ZIP
# and, if Inno Setup is present, an installer .exe.
param([string]$Version)
$ErrorActionPreference = "Stop"
$SRC = $PSScriptRoot
if (-not $Version) {
  $cml = Get-Content (Join-Path $SRC 'CMakeLists.txt') -Raw
  if ($cml -match 'project\s*\([^)]*VERSION\s+(\d+\.\d+\.\d+)') {
    $Version = "$($Matches[1])-dev"
  } else {
    throw "could not parse VERSION from CMakeLists.txt"
  }
}

$DIST = Join-Path $SRC "dist"
$ROOT = Join-Path $DIST "stage\obs-nyan-real-3dof"
$bin  = Join-Path $ROOT "bin\64bit"
$data = Join-Path $ROOT "data"
$dll  = Join-Path $SRC "build\obs-nyan-real-3dof.dll"
if (-not (Test-Path $dll)) { throw "build first (build\obs-nyan-real-3dof.dll missing)" }

Remove-Item (Join-Path $DIST "stage") -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $bin, $data | Out-Null

Copy-Item $dll $bin -Force
Copy-Item (Join-Path $SRC "data\*") $data -Recurse -Force
# docs/ is developer-only documentation (Japanese) and is intentionally not
# shipped to end users, except docs/images which the READMEs embed; only
# user-facing files go into the distribution.
foreach ($f in @("LICENSE", "THIRD_PARTY_LICENSES", "README.md", "README.en.md")) {
  Copy-Item (Join-Path $SRC $f) $ROOT -Force
}
New-Item -ItemType Directory -Force (Join-Path $ROOT "docs") | Out-Null
Copy-Item (Join-Path $SRC "docs\images") (Join-Path $ROOT "docs\images") -Recurse -Force

$zip = Join-Path $DIST "obs-nyan-real-3dof-$Version-windows-x64.zip"
Compress-Archive -Path $ROOT -DestinationPath $zip -Force
Write-Host "zip      -> $zip" -ForegroundColor Green

$iscc = @(
  "C:\Program Files (x86)\Inno Setup 6\ISCC.exe",
  "C:\Program Files\Inno Setup 6\ISCC.exe"
) | Where-Object { Test-Path $_ } | Select-Object -First 1
if ($iscc) {
  & $iscc "/DAppVersion=$Version" (Join-Path $SRC "installer\obs-nyan-real-3dof.iss")
  if ($LASTEXITCODE -ne 0) { throw "ISCC failed ($LASTEXITCODE)" }
  Write-Host "installer-> $(Get-ChildItem $DIST -Filter "*installer.exe" | Select-Object -Expand FullName)" -ForegroundColor Green
} else {
  Write-Warning "Inno Setup (ISCC.exe) not found - skipped installer (ZIP still built)."
}

Write-Host "`n=== artifacts ===" -ForegroundColor Green
Get-ChildItem $DIST -File | Select-Object Name, Length
