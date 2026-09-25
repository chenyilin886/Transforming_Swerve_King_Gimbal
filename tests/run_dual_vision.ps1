$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$exe = Join-Path ([System.IO.Path]::GetTempPath()) ('dual_vision_' + [guid]::NewGuid().ToString('N') + '.exe')
$protocolExe = Join-Path ([System.IO.Path]::GetTempPath()) ('lk_protocol_' + [guid]::NewGuid().ToString('N') + '.exe')
Push-Location $repo
try {
    & g++ -std=c++17 -Wall -Wextra -Itests/stubs -IApplication -IAlgorithm -IBSP tests/dual_vision_modes.cpp Application/ShootFSM.cpp Algorithm/PID.cpp -o $exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw 'Host regression test failed' }
    & g++ -std=c++17 -Wall -Wextra -Itests/stubs -IBSP/Motor -IBSP/Common/StateWatch -IHAL/CAN tests/lk4005_protocol.cpp BSP/Common/StateWatch/state_watch.cpp -o $protocolExe
    if ($LASTEXITCODE -ne 0) { throw 'LK protocol test compilation failed' }
    & $protocolExe
    if ($LASTEXITCODE -ne 0) { throw 'LK protocol regression test failed' }
} finally {
    Pop-Location
    if (Test-Path -LiteralPath $exe) { Remove-Item -LiteralPath $exe }
    if (Test-Path -LiteralPath $protocolExe) { Remove-Item -LiteralPath $protocolExe }
}
