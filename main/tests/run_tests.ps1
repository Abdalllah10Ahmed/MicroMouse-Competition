$ErrorActionPreference = "Stop"
$testDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Split-Path -Parent $testDir
$binary = Join-Path $env:TEMP "micromouse_maze_map_test.exe"

& g++ -std=c++17 -Wall -Wextra -pedantic `
    "-I$(Join-Path $testDir 'fakes')" `
    (Join-Path $testDir "maze_map_test.cpp") `
    (Join-Path $root "MazeMap.cpp") `
    -o $binary
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $binary
exit $LASTEXITCODE
