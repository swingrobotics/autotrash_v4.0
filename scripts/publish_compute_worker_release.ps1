param(
    [string]$Version = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $Root

if (-not $Version) {
    $Version = (Get-Content ".github/releases/compute-worker-version.txt" -Raw).Trim()
}
if ($Version -notmatch '^\d+\.\d+\.\d+$') {
    throw "Invalid release version: $Version"
}

$Repository = "swingrobotics/autotrash_v4.0"
$Tag = "compute-worker-v$Version"
$Installer = Join-Path $Root "installer-dist\SWING-Compute-Worker-Setup.exe"
$HashFile = "$Installer.sha256"
$Notes = Join-Path $Root "installer-dist\release-notes.md"

if (-not (Test-Path $Installer)) {
    throw "Installer not found: $Installer"
}
if (-not (Test-Path $HashFile)) {
    throw "SHA256 file not found: $HashFile"
}
if ($null -eq (Get-Command "gh" -ErrorAction SilentlyContinue)) {
    throw "GitHub CLI (gh) is required. Install it with: winget install --id GitHub.cli -e"
}

# Use cmd.exe for probe commands so Windows PowerShell 5.1 does not convert
# expected gh stderr (for example, 'release not found') into a terminating error
# when ErrorActionPreference is Stop.
& cmd.exe /d /c "gh auth status >NUL 2>NUL"
if ($LASTEXITCODE -ne 0) {
    & gh auth login --hostname github.com --git-protocol https --web
    if ($LASTEXITCODE -ne 0) {
        throw "GitHub CLI login failed."
    }
}

@"
## SWING Compute Worker $Version

Windows Compute Worker installer for the public SWING_CAR source release.

- Synchronized RECORD model preview with H.264 dashboard playback.
- AUTO_GPS temporal training with recent IMU yaw-rate history.
- Encoder-measured steering history for temporal AUTO_GPS models.
- Rare curve/recovery retention and curve-aware training metrics.
- AUTO_AI/AUTO_GPS training, ONNX export, model preview and Worker Manager.
- RECORD preview has no vehicle control authority (`CONTROL NONE`).
"@ | Set-Content -Path $Notes -Encoding UTF8

& cmd.exe /d /c "gh release view $Tag --repo $Repository >NUL 2>NUL"
$ReleaseExists = ($LASTEXITCODE -eq 0)

if ($ReleaseExists) {
    & gh release upload $Tag $Installer $HashFile --clobber --repo $Repository
    if ($LASTEXITCODE -ne 0) {
        throw "GitHub Release asset upload failed with exit code $LASTEXITCODE"
    }
} else {
    & gh release create $Tag $Installer $HashFile `
        --target main `
        --title "SWING Compute Worker $Version" `
        --notes-file $Notes `
        --repo $Repository
    if ($LASTEXITCODE -ne 0) {
        throw "GitHub Release creation failed with exit code $LASTEXITCODE"
    }
}

Write-Host "Published GitHub Release: $Tag" -ForegroundColor Green
