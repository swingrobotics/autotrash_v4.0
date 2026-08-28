@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\install_terminal_protocol.ps1"
if errorlevel 1 (
  echo.
  echo SWING terminal protocol install failed.
  pause
)
endlocal
