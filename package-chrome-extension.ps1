# Package the Chrome extension (tools/chrome-extension) into an upload zip for
# the Chrome Web Store / Edge Add-ons: manifest.json sits at the ZIP ROOT (a
# store requirement), with README and dotfiles excluded. The version comes from
# the extension's own manifest.json. ASCII-only so it parses under PS 5.1.
param([string]$OutDir)
$ErrorActionPreference = "Stop"
$SRC = $PSScriptRoot
$EXT = Join-Path $SRC "tools\chrome-extension"
$manifestPath = Join-Path $EXT "manifest.json"
if (-not (Test-Path $manifestPath)) { throw "manifest not found: $manifestPath" }

$manifest = Get-Content $manifestPath -Raw | ConvertFrom-Json
$ver = $manifest.version
if (-not $ver) { throw "could not read 'version' from manifest.json" }

if (-not $OutDir) { $OutDir = Join-Path $SRC "dist" }
New-Item -ItemType Directory -Force $OutDir | Out-Null

# Stage under TEMP (outside the Dropbox-synced repo) so Compress-Archive can't be
# blocked by a sync lock mid-zip. Exclude README and dotfiles from the upload.
$STAGE = Join-Path $env:TEMP ("nyan-ext-" + [Guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Force $STAGE | Out-Null
# Exclude docs (*.md: README, PUBLISHING*) and dotfiles from the store upload.
$exclude = @(".DS_Store", "Thumbs.db", "desktop.ini")
Get-ChildItem -LiteralPath $EXT -Force |
  Where-Object { $_.Name -notlike ".*" -and $_.Name -notlike "*.md" -and $exclude -notcontains $_.Name } |
  ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $STAGE -Recurse -Force }

if (-not (Test-Path (Join-Path $STAGE "manifest.json"))) {
  throw "staging failed: manifest.json missing from the zip root"
}

$zipName = "chrome-extension-$ver.zip"
$tmpzip = Join-Path $env:TEMP $zipName
Remove-Item $tmpzip -Force -ErrorAction SilentlyContinue
# Zip the CONTENTS of the stage so manifest.json lands at the archive root.
Compress-Archive -Path (Join-Path $STAGE "*") -DestinationPath $tmpzip -Force

$out = Join-Path $OutDir $zipName
for ($i = 0; $i -lt 5; $i++) {
  try { Copy-Item $tmpzip $out -Force; break }
  catch { if ($i -eq 4) { throw }; Start-Sleep -Milliseconds 500 }
}
Remove-Item $STAGE -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item $tmpzip -Force -ErrorAction SilentlyContinue

Write-Host "store zip -> $out  (manifest version $ver)" -ForegroundColor Green
# Show the archive layout; manifest.json must appear at the root (no parent dir).
Add-Type -AssemblyName System.IO.Compression.FileSystem
$z = [System.IO.Compression.ZipFile]::OpenRead($out)
$z.Entries | ForEach-Object { "  $($_.FullName)" } | Sort-Object
$z.Dispose()
