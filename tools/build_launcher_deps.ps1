<#
build_launcher_deps.ps1 — build RmlUi (core) + FreeType static libs for the
hand-maintained MSBuild game projects (smw.vcxproj et al.) to link against.

Configures lib/CMakeLists.txt with the Visual Studio generator (multi-config)
and builds the requested configurations. Outputs land under lib/_build/ in the
per-config archive directories; this script copies the two libs we care about
into lib/_build/out/<Config>/ for a stable, predictable link path.

Usage (from snesrecomp/ or anywhere):
  powershell -ExecutionPolicy Bypass -File tools\build_launcher_deps.ps1
  powershell -File tools\build_launcher_deps.ps1 -Config Debug
  powershell -File tools\build_launcher_deps.ps1 -Config Both
#>
param(
  [ValidateSet('Release','Debug','Both')]
  [string]$Config = 'Release',
  [string]$Generator = 'Visual Studio 17 2022'
)
$ErrorActionPreference = 'Stop'

$root    = Split-Path -Parent $PSScriptRoot           # snesrecomp/
$libDir  = Join-Path $root 'lib'
$buildDir= Join-Path $libDir '_build'
$outDir  = Join-Path $buildDir 'out'

# Prefer the Visual Studio-bundled cmake (matches the VS generator/toolset
# cleanly); fall back to whatever `cmake` is on PATH.
$cmake = 'cmake'
$vsCmake = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if (Test-Path $vsCmake) { $cmake = $vsCmake }

if (-not (Test-Path (Join-Path $libDir 'RmlUi\CMakeLists.txt')) -or
    -not (Test-Path (Join-Path $libDir 'freetype\CMakeLists.txt'))) {
  throw "lib/RmlUi or lib/freetype missing. Run: git submodule update --init --recursive"
}

Write-Host "[deps] configuring ($Generator, x64)..." -ForegroundColor Cyan
& $cmake -S $libDir -B $buildDir -G $Generator -A x64
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

$configs = if ($Config -eq 'Both') { @('Release','Debug') } else { @($Config) }
foreach ($c in $configs) {
  Write-Host "[deps] building $c..." -ForegroundColor Cyan
  & $cmake --build $buildDir --config $c --target rmlui_core freetype
  if ($LASTEXITCODE -ne 0) { throw "cmake build ($c) failed" }

  $dest = Join-Path $outDir $c
  New-Item -ItemType Directory -Force -Path $dest | Out-Null
  $libs = Get-ChildItem -Path $buildDir -Recurse -Filter *.lib |
          Where-Object { $_.FullName -match "\\$c\\" -and
                         ($_.Name -match 'rmlui|RmlCore|freetype') }
  foreach ($l in $libs) { Copy-Item $l.FullName $dest -Force }
  Write-Host "[deps] $c libs -> $dest" -ForegroundColor Green
  Get-ChildItem $dest -Filter *.lib | ForEach-Object { "  $($_.Name)" }
}
Write-Host "[deps] done." -ForegroundColor Green
