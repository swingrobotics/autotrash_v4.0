@echo off
setlocal
set "ROOT=%~dp0..\.."
cd /d "%ROOT%"

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%ROOT%\scripts\install_terminal_protocol.ps1" -Quiet
if errorlevel 1 (
  echo.
  echo SWING terminal protocol install failed.
  pause
  exit /b 1
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%ROOT%\scripts\install_compute_worker_dev.ps1"
if errorlevel 1 (
  echo.
  echo SWING Compute Worker DEV install failed.
  pause
)
endlocal
