$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$exe = Join-Path ([System.IO.Path]::GetTempPath()) ('dual_vision_' + [guid]::NewGuid().ToString('N') + '.exe')
Push-Location $repo
try {
    & g++ -std=c++17 -Wall -Wextra -Itests/stubs -IApplication -IAlgorithm -IBSP tests/dual_vision_modes.cpp Application/ShootFSM.cpp Algorithm/PID.cpp -o $exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw 'Host regression test failed' }
} finally {
    Pop-Location
    if (Test-Path -LiteralPath $exe) { Remove-Item -LiteralPath $exe }
}
