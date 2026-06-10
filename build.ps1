# Configure + build obs-nyan-real-3dof with the VS18-bundled CMake/Ninja + MSVC.
$ErrorActionPreference = "Stop"
$SRC   = $PSScriptRoot
$BUILD = "$SRC\build"

function Find-VsPath {
  $preferred = "C:\Program Files\Microsoft Visual Studio\18\Community"
  if (Test-Path $preferred) { return $preferred }
  $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
  if (Test-Path $vswhere) {
    $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($vsPath -and (Test-Path $vsPath)) { return $vsPath }
  }
  throw "Visual Studio with MSVC x64 tools not found."
}

function Find-Exe {
  param([string[]]$Candidates, [string]$Name)
  foreach ($c in $Candidates) {
    if ($c -and (Test-Path $c)) { return $c }
  }
  $cmd = Get-Command $Name -ErrorAction SilentlyContinue
  if ($cmd) { return $cmd.Source }
  throw "$Name not found."
}

$VS = Find-VsPath
$VCV = Join-Path $VS "VC\Auxiliary\Build\vcvars64.bat"
$CMAKE = Find-Exe @(
  (Join-Path $VS "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe")
) "cmake"
$NINJA = Find-Exe @(
  (Join-Path $VS "Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe")
) "ninja"

$cfg = "`"$CMAKE`" -G Ninja -S `"$SRC`" -B `"$BUILD`" " +
       "-DCMAKE_MAKE_PROGRAM=`"$NINJA`" -DCMAKE_C_COMPILER=cl -DCMAKE_CXX_COMPILER=cl " +
       "-DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY " +
       "-DCMAKE_BUILD_TYPE=Release"
$bld = "`"$CMAKE`" --build `"$BUILD`""

cmd /c "`"$VCV`" && $cfg && $bld"
if ($LASTEXITCODE -ne 0) { throw "build failed ($LASTEXITCODE)" }
Write-Host "`n=== built artifacts ===" -ForegroundColor Green
Get-ChildItem "$BUILD\*.dll" | Select-Object FullName, Length
