param(
    [string]$Version = "",
    [switch]$Publish
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $Root

# Python launched as `python path\to\script.py` places that script directory at
# sys.path[0], not necessarily the repository root. GitHub Download ZIP users do
# not have an editable install, so make the source tree importable explicitly for
# every smoke/build subprocess.
if ($env:PYTHONPATH) {
    $env:PYTHONPATH = "$Root;$env:PYTHONPATH"
} else {
    $env:PYTHONPATH = $Root
}

function Invoke-Checked {
    param([string]$Label, [scriptblock]$Action)
    Write-Host "`n==> $Label" -ForegroundColor Cyan
    & $Action
    if ($LASTEXITCODE -ne 0) {
        throw "$Label failed with exit code $LASTEXITCODE"
    }
}

function Command-Exists {
    param([string]$Name)
    return $null -ne (Get-Command $Name -ErrorAction SilentlyContinue)
}

function Refresh-Path {
    $machine = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $user = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$machine;$user"
}

function Resolve-InnoSetup {
    $command = Get-Command "ISCC.exe" -ErrorAction SilentlyContinue
    if ($command -and (Test-Path $command.Source)) {
        return $command.Source
    }

    $candidates = @(
        "C:\Program Files (x86)\Inno Setup 6\ISCC.exe",
        "C:\Program Files\Inno Setup 6\ISCC.exe",
        (Join-Path $env:LOCALAPPDATA "Programs\Inno Setup 6\ISCC.exe"),
        (Join-Path $env:LOCALAPPDATA "Inno Setup 6\ISCC.exe")
    )
    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path $candidate)) {
            return (Resolve-Path $candidate).Path
        }
    }

    $registryKeys = @(
        "HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 6_is1",
        "HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 6_is1",
        "HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 6_is1"
    )
    foreach ($key in $registryKeys) {
        try {
            $item = Get-ItemProperty -Path $key -ErrorAction Stop
            $installLocation = [string]$item.InstallLocation
            if ($installLocation) {
                $candidate = Join-Path $installLocation "ISCC.exe"
                if (Test-Path $candidate) {
                    return (Resolve-Path $candidate).Path
                }
            }
        } catch {
            # Registry key is optional; continue with filesystem discovery.
        }
    }

    $roots = @(
        $env:ProgramFiles,
        ${env:ProgramFiles(x86)},
        (Join-Path $env:LOCALAPPDATA "Programs"),
        (Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages")
    ) | Where-Object { $_ -and (Test-Path $_) }
    foreach ($searchRoot in $roots) {
        $found = Get-ChildItem -Path $searchRoot -Filter "ISCC.exe" -File -Recurse -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($found) {
            return $found.FullName
        }
    }
    return $null
}

function Ensure-InnoSetup {
    $resolved = Resolve-InnoSetup
    if ($resolved) {
        Write-Host "Using Inno Setup compiler: $resolved" -ForegroundColor DarkGray
        return $resolved
    }

    if (Command-Exists "winget") {
        Write-Host "`nInno Setup 6 not found. Installing with winget..." -ForegroundColor Yellow
        & winget install --id JRSoftware.InnoSetup -e --accept-package-agreements --accept-source-agreements
        if ($LASTEXITCODE -ne 0) {
            throw "Could not install Inno Setup 6 with winget. Run: winget install --id JRSoftware.InnoSetup -e"
        }
        Refresh-Path
        Start-Sleep -Seconds 1
        $resolved = Resolve-InnoSetup
    }
    if (-not $resolved) {
        throw "Inno Setup 6 installation completed but ISCC.exe could not be located. Reopen PowerShell or install Inno Setup 6 manually, then run this command again."
    }
    Write-Host "Using Inno Setup compiler: $resolved" -ForegroundColor DarkGray
    return $resolved
}

function Ensure-GitHubCli {
    if (Command-Exists "gh") { return }
    if (-not (Command-Exists "winget")) {
        throw "GitHub CLI (gh) is required for -Publish and winget is unavailable. Install GitHub CLI, then run again."
    }
    Write-Host "`nGitHub CLI not found. Installing with winget..." -ForegroundColor Yellow
    & winget install --id GitHub.cli -e --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) {
        throw "Could not install GitHub CLI with winget."
    }
    Refresh-Path
    if (-not (Command-Exists "gh")) {
        throw "GitHub CLI was installed but is not available in PATH. Reopen PowerShell and run again."
    }
}

if (-not $Version) {
    $Version = (Get-Content ".github/releases/compute-worker-version.txt" -Raw).Trim()
}
if ($Version -notmatch '^\d+\.\d+\.\d+$') {
    throw "Invalid release version: $Version"
}

# Git is optional. A GitHub 'Download ZIP' archive intentionally contains no
# .git directory, so release builds must also work from a verified main archive.
$GitAvailable = Command-Exists "git"
$GitRepository = $false
$Head = "main"
$SourceDescription = "main source archive"
if ($GitAvailable) {
    & git rev-parse --is-inside-work-tree *> $null
    if ($LASTEXITCODE -eq 0) {
        $GitRepository = $true
        $Branch = (& git rev-parse --abbrev-ref HEAD).Trim()
        if ($LASTEXITCODE -ne 0) { throw "Could not determine git branch" }
        if ($Branch -ne "main") {
            throw "Release builds must be made from main. Current branch: $Branch"
        }
        $Head = (& git rev-parse HEAD).Trim()
        if ($LASTEXITCODE -ne 0 -or -not $Head) { throw "Could not determine git commit" }
        $SourceDescription = "main commit $Head"
    }
}
if (-not $GitRepository) {
    Write-Host "Git repository not detected. Building directly from the downloaded main source archive." -ForegroundColor Yellow
}

Invoke-Checked "Verify Python" { python --version }
Invoke-Checked "Install/update Worker build dependencies" {
    python -m pip install --disable-pip-version-check -r requirements-compute-worker.txt
}
Invoke-Checked "Compile source" {
    python -m compileall -q swing_compute autonomous_car scripts/run_compute_worker.py scripts/run_compute_manager.py
}
Invoke-Checked "Worker CLI smoke" { python scripts/run_compute_worker.py --help }
Invoke-Checked "Temporal GPS contract" { python -m autonomous_car.simulation.validate_temporal_gps_training_v2 }
Invoke-Checked "RECORD preview smoke" { python -m autonomous_car.simulation.validate_record_model_preview_v2 }
Invoke-Checked "Dashboard preview contract" { python scripts/validate_record_preview_dashboard.py }

Remove-Item -Recurse -Force build, dist, installer-dist -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force installer-dist | Out-Null

Invoke-Checked "Build SWING Compute Worker" {
    python -m PyInstaller `
        --noconfirm `
        --clean `
        --onedir `
        --windowed `
        --name SWING-Compute-Worker `
        --collect-all openvino `
        --hidden-import autonomous_car.ai `
        --hidden-import autonomous_car.ai.measured_steering_gps `
        --hidden-import autonomous_car.ai.record_preview `
        --hidden-import swing_compute.record_preview_worker_extensions `
        scripts/run_compute_worker.py
}
$WorkerExe = Join-Path $Root "dist\SWING-Compute-Worker\SWING-Compute-Worker.exe"
if (-not (Test-Path $WorkerExe)) { throw "Worker executable was not produced: $WorkerExe" }

Invoke-Checked "Build SWING Compute Manager" {
    python -m PyInstaller `
        --noconfirm `
        --clean `
        --onefile `
        --windowed `
        --name SWING-Compute-Manager `
        --exclude-module swing_compute.pipeline_worker `
        --exclude-module swing_compute.worker `
        --exclude-module autonomous_car `
        scripts/run_compute_manager.py
}
$ManagerExe = Join-Path $Root "dist\SWING-Compute-Manager.exe"
if (-not (Test-Path $ManagerExe)) { throw "Manager executable was not produced: $ManagerExe" }

$Iscc = Ensure-InnoSetup
Invoke-Checked "Build installer EXE" {
    & $Iscc "/DAppVersion=$Version" "packaging\windows\SWINGComputeWorker.iss"
}

$Installer = Join-Path $Root "installer-dist\SWING-Compute-Worker-Setup.exe"
$HashFile = "$Installer.sha256"
if (-not (Test-Path $Installer)) { throw "Installer was not produced: $Installer" }
$Hash = (Get-FileHash $Installer -Algorithm SHA256).Hash
"$Hash  SWING-Compute-Worker-Setup.exe" | Set-Content -Path $HashFile -Encoding Ascii

Write-Host "`nBuilt:" -ForegroundColor Green
Write-Host "  $Installer"
Write-Host "  $HashFile"
Write-Host "  SHA256 $Hash"

if ($Publish) {
    Ensure-GitHubCli
    & gh auth status *> $null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "`nGitHub login is required once. A browser login will start now." -ForegroundColor Yellow
        & gh auth login --hostname github.com --git-protocol https --web
        if ($LASTEXITCODE -ne 0) {
            throw "GitHub CLI login failed. The installer was still built successfully at $Installer"
        }
    }

    $Repository = "swingrobotics/autotrash_v4.0"
    $Tag = "compute-worker-v$Version"
    $Notes = Join-Path $Root "installer-dist\release-notes.md"
    @"
## SWING Compute Worker $Version

Windows Compute Worker installer built from $SourceDescription.

- Synchronized RECORD model preview with H.264 dashboard playback.
- AUTO_GPS temporal training with recent IMU yaw-rate history.
- Encoder-measured steering history for new temporal AUTO_GPS models.
- Rare curve/recovery retention and curve-aware training metrics.
- AUTO_AI/AUTO_GPS training, ONNX export, model preview and Worker Manager.
- RECORD preview has no vehicle control authority (`CONTROL NONE`).
"@ | Set-Content -Path $Notes -Encoding UTF8

    & gh release view $Tag --repo $Repository *> $null
    if ($LASTEXITCODE -eq 0) {
        Invoke-Checked "Upload release assets" {
            gh release upload $Tag $Installer $HashFile --clobber --repo $Repository
        }
    } else {
        Invoke-Checked "Create GitHub Release $Tag" {
            gh release create $Tag $Installer $HashFile `
                --target main `
                --title "SWING Compute Worker $Version" `
                --notes-file $Notes `
                --repo $Repository
        }
    }
    Write-Host "`nPublished GitHub Release: $Tag" -ForegroundColor Green
}
