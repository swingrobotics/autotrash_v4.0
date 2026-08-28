$ErrorActionPreference = 'Stop'

function Write-Step([string]$Message) {
    Write-Host "[SWING] $Message" -ForegroundColor Cyan
}

function Test-Administrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

if (-not (Test-Administrator)) {
    Write-Step 'Requesting administrator privileges...'
    $arguments = "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    Start-Process powershell.exe -Verb RunAs -ArgumentList $arguments
    exit 0
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$installRoot = Join-Path $env:LOCALAPPDATA 'SWING Robotics\Compute Worker Dev'
$appRoot = Join-Path $installRoot 'app'
$venvRoot = Join-Path $installRoot 'venv'
$dataRoot = Join-Path $env:LOCALAPPDATA 'SWING Robotics\Compute Worker'
$startup = [Environment]::GetFolderPath('Startup')
$startupShortcut = Join-Path $startup 'SWING Compute Worker Dev.lnk'
$statusShortcut = Join-Path ([Environment]::GetFolderPath('Desktop')) 'SWING Compute Worker Status.lnk'
$stdoutLog = Join-Path $installRoot 'worker.stdout.log'
$stderrLog = Join-Path $installRoot 'worker.stderr.log'

function Resolve-Python311 {
    $commands = @(
        @{ File = 'py'; Args = @('-3.11') },
        @{ File = 'python'; Args = @() }
    )
    foreach ($candidate in $commands) {
        try {
            $version = & $candidate.File @($candidate.Args) -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>$null
            if ($LASTEXITCODE -eq 0 -and $version.Trim() -eq '3.11') {
                $exe = & $candidate.File @($candidate.Args) -c "import sys; print(sys.executable)"
                return $exe.Trim()
            }
        } catch {}
    }

    $known = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe'),
        'C:\Python311\python.exe'
    )
    foreach ($path in $known) {
        if (Test-Path $path) { return $path }
    }
    return $null
}

function Stop-ExistingSwingWorker {
    $verifiedWorker = $false
    try {
        $health = Invoke-RestMethod -Uri 'http://127.0.0.1:8765/api/v1/health' -TimeoutSec 2
        $verifiedWorker = ($health.service -eq 'swing-compute-worker')
    } catch {}

    if ($verifiedWorker) {
        $listeners = Get-NetTCPConnection -LocalPort 8765 -State Listen -ErrorAction SilentlyContinue
        foreach ($listener in $listeners) {
            if ($listener.OwningProcess -and $listener.OwningProcess -ne $PID) {
                Stop-Process -Id $listener.OwningProcess -Force -ErrorAction SilentlyContinue
            }
        }
    }

    # Also stop a stale DEV worker even if its HTTP listener is already unhealthy.
    # Match only our copied run_compute_worker.py under the DEV install tree so an
    # unrelated Python process is never terminated.
    try {
        $processes = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue | Where-Object {
            $_.ProcessId -ne $PID -and
            $_.CommandLine -and
            $_.CommandLine -like '*run_compute_worker.py*' -and
            $_.CommandLine -like '*SWING Robotics*Compute Worker Dev*'
        }
        foreach ($process in $processes) {
            Stop-Process -Id $process.ProcessId -Force -ErrorAction SilentlyContinue
        }
    } catch {}

    for ($i = 0; $i -lt 20; $i++) {
        $lockedByWorker = $false
        try {
            $lockedByWorker = [bool](Get-CimInstance Win32_Process -ErrorAction SilentlyContinue | Where-Object {
                $_.CommandLine -and
                $_.CommandLine -like '*run_compute_worker.py*' -and
                $_.CommandLine -like '*SWING Robotics*Compute Worker Dev*'
            } | Select-Object -First 1)
        } catch {}
        if (-not $lockedByWorker) { break }
        Start-Sleep -Milliseconds 250
    }
}

function Remove-AppTreeWithRetry {
    if (-not (Test-Path $appRoot)) { return }
    $lastError = $null
    for ($attempt = 1; $attempt -le 12; $attempt++) {
        try {
            Remove-Item $appRoot -Recurse -Force -ErrorAction Stop
            return
        } catch {
            $lastError = $_
            Start-Sleep -Milliseconds 500
        }
    }
    throw "Unable to replace the previous Worker app because Windows still has it open. Close any SWING/Python windows and run the installer again. Last error: $($lastError.Exception.Message)"
}

$python = Resolve-Python311
if (-not $python) {
    if (-not (Get-Command winget.exe -ErrorAction SilentlyContinue)) {
        throw 'Python 3.11 was not found and winget is unavailable. Install Python 3.11 x64 and run this installer again.'
    }
    Write-Step 'Installing Python 3.11...'
    winget install --id Python.Python.3.11 -e --scope user --accept-package-agreements --accept-source-agreements --silent
    if ($LASTEXITCODE -ne 0) { throw "Python 3.11 installation failed: exit=$LASTEXITCODE" }
    $python = Resolve-Python311
    if (-not $python) { throw 'Python 3.11 was installed but could not be located. Sign out/in to Windows and run this installer again.' }
}

Write-Step "Python: $python"
Write-Step 'Stopping any previous SWING Compute Worker...'
Stop-ExistingSwingWorker

Write-Step 'Copying SWING Compute Worker files...'
New-Item -ItemType Directory -Path $installRoot -Force | Out-Null
Remove-AppTreeWithRetry
New-Item -ItemType Directory -Path $appRoot -Force | Out-Null
Copy-Item (Join-Path $repoRoot 'swing_compute') $appRoot -Recurse -Force
Copy-Item (Join-Path $repoRoot 'autonomous_car') $appRoot -Recurse -Force
# Live UFLD imports the vendored decoder as third_party.ufld. Copy the whole
# third_party tree so the DEV install has the same inference contract, license
# files and decoder implementation as the source checkout.
Copy-Item (Join-Path $repoRoot 'third_party') $appRoot -Recurse -Force
New-Item -ItemType Directory -Path (Join-Path $appRoot 'scripts') -Force | Out-Null
Copy-Item (Join-Path $repoRoot 'scripts\run_compute_worker.py') (Join-Path $appRoot 'scripts\run_compute_worker.py') -Force
Copy-Item (Join-Path $repoRoot 'requirements-ai-training.txt') $appRoot -Force
Copy-Item (Join-Path $repoRoot 'requirements-compute-worker.txt') $appRoot -Force

$ufldDecoder = Join-Path $appRoot 'third_party\ufld\decoder.py'
if (-not (Test-Path $ufldDecoder)) {
    throw "UFLD decoder was not copied into the DEV Worker app: $ufldDecoder"
}

if (-not (Test-Path (Join-Path $venvRoot 'Scripts\python.exe'))) {
    Write-Step 'Creating the dedicated Python environment...'
    & $python -m venv $venvRoot
    if ($LASTEXITCODE -ne 0) { throw 'Virtual environment creation failed.' }
}

$venvPython = Join-Path $venvRoot 'Scripts\python.exe'
$venvPythonw = Join-Path $venvRoot 'Scripts\pythonw.exe'
$runner = Join-Path $appRoot 'scripts\run_compute_worker.py'

Write-Step 'Installing/updating PyTorch CPU, OpenVINO and ONNX Runtime. The first install can take a while...'
& $venvPython -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { throw 'pip upgrade failed.' }
& $venvPython -m pip install -r (Join-Path $appRoot 'requirements-compute-worker.txt')
if ($LASTEXITCODE -ne 0) { throw 'Compute Worker dependency installation failed.' }

Write-Step 'Checking Worker Python imports...'
& $venvPython $runner --help | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Worker import/self-test failed before startup.' }

Write-Step 'Configuring the Windows Private-network firewall rule...'
Get-NetFirewallRule -DisplayName 'SWING Compute Worker Dev' -ErrorAction SilentlyContinue | Remove-NetFirewallRule -ErrorAction SilentlyContinue
New-NetFirewallRule -DisplayName 'SWING Compute Worker Dev' -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8765 -Profile Private -Program $venvPythonw | Out-Null

Write-Step 'Starting SWING Compute Worker...'
Remove-Item $stdoutLog, $stderrLog -Force -ErrorAction SilentlyContinue
$env:SWING_COMPUTE_DATA_ROOT = $dataRoot
$quotedRunner = '"' + $runner + '"'
Start-Process -FilePath $venvPython -ArgumentList @('-u', $quotedRunner, '--background') -WorkingDirectory $appRoot -WindowStyle Hidden -RedirectStandardOutput $stdoutLog -RedirectStandardError $stderrLog

$healthy = $false
for ($i = 0; $i -lt 40; $i++) {
    Start-Sleep -Milliseconds 500
    try {
        $health = Invoke-RestMethod -Uri 'http://127.0.0.1:8765/api/v1/health' -TimeoutSec 2
        if ($health.service -eq 'swing-compute-worker') { $healthy = $true; break }
    } catch {}
}

if (-not $healthy) {
    Write-Host ''
    Write-Host 'Worker startup failed. Diagnostic log:' -ForegroundColor Red
    if (Test-Path $stderrLog) { Get-Content $stderrLog -Tail 80 }
    if (Test-Path $stdoutLog) { Get-Content $stdoutLog -Tail 40 }
    throw "Worker startup/health check failed. Logs: $stderrLog"
}

$wsh = New-Object -ComObject WScript.Shell
$shortcut = $wsh.CreateShortcut($startupShortcut)
$shortcut.TargetPath = $venvPythonw
$shortcut.Arguments = '"' + $runner + '" --background'
$shortcut.WorkingDirectory = $appRoot
$shortcut.Description = 'SWING Compute Worker Dev auto start'
$shortcut.Save()

$status = $wsh.CreateShortcut($statusShortcut)
$status.TargetPath = "$env:WINDIR\explorer.exe"
$status.Arguments = 'http://127.0.0.1:8765/'
$status.Description = 'Open SWING Compute Worker status'
$status.Save()

$statusPayload = $null
for ($i = 0; $i -lt 8; $i++) {
    try {
        $statusPayload = Invoke-RestMethod -Uri 'http://127.0.0.1:8765/api/v1/status' -TimeoutSec 5
        if ($statusPayload.service -eq 'swing-compute-worker') { break }
    } catch {
        Start-Sleep -Milliseconds 500
    }
}

Write-Host ''
Write-Host 'SWING Compute Worker DEV installation complete.' -ForegroundColor Green
if ($statusPayload) {
    Write-Host "  Version : $($statusPayload.version)"
    Write-Host "  PC      : $($statusPayload.hostname)"
} else {
    Write-Host '  Version : Worker is healthy; detailed status is still warming up.'
}
Write-Host '  Port    : 8765'
Write-Host "  Data    : $dataRoot"
Write-Host '  Status  : http://127.0.0.1:8765/'
Write-Host "  Log     : $stderrLog"
Write-Host ''
Write-Host 'Connect the Raspberry Pi over the private LAN, then check Compute PC status in the rover Settings/Data page.' -ForegroundColor Green
Start-Process 'http://127.0.0.1:8765/'
Read-Host 'Press Enter to close this installer window'
