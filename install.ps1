# Install obs-nyan-real-3dof into the per-user OBS plugin folder (no admin required).
$ErrorActionPreference = "Stop"
$SRC = $PSScriptRoot
$dll    = "$SRC\build\obs-nyan-real-3dof.dll"
$effect = "$SRC\data\nyan-real-3dof.effect"

$required = [ordered]@{
  "plugin DLL (run build.ps1)" = $dll
  "3DoF effect"               = $effect
}
$missing = $required.GetEnumerator() | Where-Object { -not (Test-Path $_.Value) }
if ($missing) {
  throw ("install aborted - missing required files:`n" +
    (($missing | ForEach-Object { "  - $($_.Key): $($_.Value)" }) -join "`n"))
}

$root = Join-Path $env:ProgramData "obs-studio\plugins\obs-nyan-real-3dof"
$bin  = Join-Path $root "bin\64bit"
$data = Join-Path $root "data"
New-Item -ItemType Directory -Force $bin, $data | Out-Null

Copy-Item $dll $bin -Force
Copy-Item "$SRC\data\*" $data -Recurse -Force

Write-Host "installed -> $bin" -ForegroundColor Green
Get-ChildItem $bin | Select-Object Name, Length

