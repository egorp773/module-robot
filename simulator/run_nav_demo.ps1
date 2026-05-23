param(
  [string]$Mode = "--random",
  [string]$A = "123",
  [string]$B = "",
  [string]$Seed = ""
)

$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$toolchain = Join-Path $repoRoot ".tools\w64devkit\bin"
$source = Join-Path $repoRoot "navigation_core\demo_random_nav.cpp"
$include = Join-Path $repoRoot "rtk_firmware\lib\NavigationCore"
$outDir = Join-Path $repoRoot ".pio\build_root"
$exe = Join-Path $outDir "nav_demo.exe"

if (-not (Test-Path $source)) {
  throw "Source file not found: $source"
}

if (-not (Test-Path (Join-Path $toolchain "g++.exe"))) {
  throw "g++ not found. Expected: $(Join-Path $toolchain 'g++.exe')"
}

New-Item -ItemType Directory -Force $outDir | Out-Null
$env:PATH = "$toolchain;$env:PATH"

g++ -std=c++17 -Wall -Wextra -I $include $source -o $exe

if ($Mode -eq "--target") {
  if ($B -eq "") {
    throw "Usage: .\run_nav_demo.ps1 --target <x_m> <y_m> [seed]"
  }

  if ($Seed -eq "") {
    & $exe --target $A $B
  } else {
    & $exe --target $A $B $Seed
  }
} elseif ($Mode -eq "--random") {
  & $exe --random $A
} else {
  throw "Usage: .\run_nav_demo.ps1 --random [seed] OR .\run_nav_demo.ps1 --target <x_m> <y_m> [seed]"
}
