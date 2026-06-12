# One-shot setup of build dependencies for obs-nyan-real-3dof.
# Safe to re-run; uses ../obs_real3d/deps as a local cache when available.
$ErrorActionPreference = "Stop"
$SRC = $PSScriptRoot
$REAL3D = Resolve-Path -LiteralPath (Join-Path $SRC "..\obs_real3d") -ErrorAction SilentlyContinue

New-Item -ItemType Directory -Force "$SRC\deps" | Out-Null

function Find-Vcvars64 {
  $preferred = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
  if (Test-Path $preferred) { return $preferred }
  $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
  if (Test-Path $vswhere) {
    $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($vsPath) {
      $candidate = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
      if (Test-Path $candidate) { return $candidate }
    }
  }
  throw "vcvars64.bat not found. Install Visual Studio with MSVC x64 tools."
}

$VCV = Find-Vcvars64

function New-JunctionOrCopy {
  param(
    [string]$Source,
    [string]$Dest
  )
  if (Test-Path $Dest) { return }
  if (-not (Test-Path $Source)) { return }
  try {
    New-Item -ItemType Junction -Path $Dest -Target $Source | Out-Null
    Write-Host "linked $Dest -> $Source" -ForegroundColor DarkGray
  } catch {
    Write-Warning "junction failed, copying instead: $($_.Exception.Message)"
    Copy-Item $Source $Dest -Recurse -Force
  }
}

# 1. libobs headers matching installed OBS -------------------------------------
$obsHeaders = "$SRC\deps\obs-studio\libobs\obs-module.h"
if (-not (Test-Path $obsHeaders)) {
  if ($REAL3D) {
    New-JunctionOrCopy -Source (Join-Path $REAL3D "deps\obs-studio") -Dest "$SRC\deps\obs-studio"
  }
}
if (-not (Test-Path $obsHeaders)) {
  $obsVer = (Get-Item "C:\Program Files\obs-studio\bin\64bit\obs64.exe").VersionInfo.ProductVersion
  if ($obsVer -match '^(\d+\.\d+\.\d+)') { $obsVer = $Matches[1] }
  Write-Host "[1/4] cloning obs-studio headers @ $obsVer"
  git clone --depth 1 --branch $obsVer --single-branch `
    https://github.com/obsproject/obs-studio.git "$SRC\deps\obs-studio"
} else {
  Write-Host "[1/4] obs headers present"
}

# 2. generated libobs config header -------------------------------------------
$obsConfig = "$SRC\deps\gen\obsconfig.h"
if (-not (Test-Path $obsConfig)) {
  Write-Host "[2/4] generating obsconfig.h"
  New-Item -ItemType Directory -Force "$SRC\deps\gen" | Out-Null
  @(
    "/*"
    " * Minimal stand-in for OBS' generated obsconfig.h."
    " * This plugin only needs enough definitions for libobs headers when building"
    " * outside the OBS source tree."
    " */"
    "#pragma once"
    ""
    "#define OBS_DATA_PATH `"../../data`""
    "#define OBS_PLUGIN_PATH `"../../obs-plugins/64bit`""
    "#define OBS_PLUGIN_DESTINATION `"obs-plugins/64bit`""
    "#define OBS_RELEASE_CANDIDATE 0"
    "#define OBS_BETA 0"
    ""
  ) | Set-Content $obsConfig -Encoding ascii
} else {
  Write-Host "[2/4] obsconfig.h present"
}

# 3. import libraries from installed OBS DLLs ----------------------------------
$obsLib = "$SRC\deps\obslib\obs.lib"
if (-not (Test-Path $obsLib)) {
  if ($REAL3D) {
    New-JunctionOrCopy -Source (Join-Path $REAL3D "deps\obslib") -Dest "$SRC\deps\obslib"
  }
}
function New-ImportLibFromDll {
  param(
    [string]$Dll,
    [string]$Lib,
    [string]$Def,
    [string]$Label
  )
  if (Test-Path $Lib) {
    Write-Host "[3/4] $Label present"
    return
  }
  Write-Host "[3/4] generating $Label"
  $tmp = "$env:TEMP\obs_3dof_$Label.exports.txt"
  cmd /c "`"$VCV`" >nul 2>&1 && dumpbin /exports `"$Dll`"" > $tmp 2>&1
  $names = foreach ($l in Get-Content $tmp) {
    if ($l -match '^\s+\d+\s+[0-9A-Fa-f]+\s+[0-9A-Fa-f]+\s+(\S+)') { $matches[1] }
  }
  $names = $names | Where-Object { $_ -and $_ -ne '=' } | Sort-Object -Unique
  New-Item -ItemType Directory -Force "$SRC\deps\obslib" | Out-Null
  @("EXPORTS") + $names | Set-Content $Def -Encoding ascii
  cmd /c "`"$VCV`" >nul 2>&1 && lib /def:`"$Def`" /out:`"$Lib`" /machine:x64 /nologo"
}

New-ImportLibFromDll `
  -Dll "C:\Program Files\obs-studio\bin\64bit\obs.dll" `
  -Lib "$SRC\deps\obslib\obs.lib" `
  -Def "$SRC\deps\obslib\obs.def" `
  -Label "obs.lib"

New-ImportLibFromDll `
  -Dll "C:\Program Files\obs-studio\bin\64bit\obs-frontend-api.dll" `
  -Lib "$SRC\deps\obslib\obs-frontend-api.lib" `
  -Def "$SRC\deps\obslib\obs-frontend-api.def" `
  -Label "obs-frontend-api.lib"

# 4. Qt headers/import libraries for the dock UI -------------------------------
$qtRoot = "$SRC\deps\qt\6.8.3\msvc2022_64"
$qtConfig = Join-Path $qtRoot "lib\cmake\Qt6\Qt6Config.cmake"
if (-not (Test-Path $qtConfig)) {
  Write-Host "[4/4] installing Qt 6.8.3 qtbase for dock UI"
  python -m pip install --user aqtinstall
  python -m aqt install-qt windows desktop 6.8.3 win64_msvc2022_64 -O "$SRC\deps\qt" --archives qtbase
} else {
  Write-Host "[4/4] Qt 6.8.3 qtbase present"
}

Write-Host "`nsetup done. Now: .\build.ps1 ; .\install.ps1" -ForegroundColor Green
