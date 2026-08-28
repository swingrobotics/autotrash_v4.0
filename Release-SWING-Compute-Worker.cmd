@echo off
setlocal
cd /d "%~dp0"

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\build_compute_worker_release.ps1"
set "EXITCODE=%ERRORLEVEL%"
if not "%EXITCODE%"=="0" goto :done

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\publish_compute_worker_release.ps1"
set "EXITCODE=%ERRORLEVEL%"

:done
echo.
if not "%EXITCODE%"=="0" (
  echo SWING Compute Worker release failed. Exit code: %EXITCODE%
) else (
  echo SWING Compute Worker release completed.
)
pause
exit /b %EXITCODE%
