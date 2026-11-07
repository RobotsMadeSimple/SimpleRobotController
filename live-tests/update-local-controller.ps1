# Update the local controller (port 9000) to the current source: stop it, rebuild the Debug build in
# place, refresh the Checkers plugin, start it again. The controller's data (programs, points, config)
# lives in the bin folder and is kept. Run from anywhere:
#   powershell -ExecutionPolicy Bypass -File SimpleRobotController\live-tests\update-local-controller.ps1
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$bin  = Join-Path $repo "RobotControl\bin\Debug\net10.0"
$exe  = Join-Path $bin "SimpleRobotController.exe"

# 1. Stop the controller that owns port 9000 — only if it is this build's exe.
$conn = Get-NetTCPConnection -State Listen -LocalPort 9000 -ErrorAction SilentlyContinue | Select-Object -First 1
if ($conn) {
  $p = Get-Process -Id $conn.OwningProcess
  if ($p.Path -ne $exe) { throw "Port 9000 is owned by $($p.Path) (PID $($p.Id)), not $exe - stopping nothing." }
  Write-Host "Stopping controller PID $($p.Id)"
  Stop-Process -Id $p.Id
  $p.WaitForExit(10000) | Out-Null
}

# 2. Rebuild in place.
Write-Host "Building RobotControl (Debug)"
dotnet build (Join-Path $repo "RobotControl") -c Debug -nologo -v q
if ($LASTEXITCODE -ne 0) { throw "Build failed - controller left stopped." }

# 3. Refresh the Checkers plugin.
$pub = Join-Path $env:TEMP "rms-checkers-pub"
Write-Host "Publishing the Checkers plugin"
dotnet publish (Join-Path $repo "examples\dotnet\Checkers") -c Release -o $pub -nologo -v q
if ($LASTEXITCODE -ne 0) { throw "Plugin publish failed - controller left stopped." }
$plug = Join-Path $bin "plugins\checkers"
New-Item -ItemType Directory -Force $plug | Out-Null
Copy-Item -Recurse -Force (Join-Path $pub "*") $plug

# 4. Start it again (working directory = bin, like before).
$new = Start-Process -FilePath $exe -WorkingDirectory $bin -PassThru
Write-Host "Controller started, PID $($new.Id)"
