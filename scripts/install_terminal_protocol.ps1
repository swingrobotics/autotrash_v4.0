param(
    [switch]$Quiet
)

$ErrorActionPreference = 'Stop'

function Write-Step([string]$Message) {
    if (-not $Quiet) { Write-Host "[SWING] $Message" -ForegroundColor Cyan }
}

$installRoot = Join-Path $env:LOCALAPPDATA 'SWING Robotics\Terminal'
$launcherSource = Join-Path $PSScriptRoot 'open_pi_terminal.ps1'
$launcherPath = Join-Path $installRoot 'open_pi_terminal.ps1'

if (-not (Test-Path $launcherSource)) {
    throw "SWING terminal launcher not found: $launcherSource"
}

Write-Step 'Installing the Windows PI terminal launcher...'
New-Item -ItemType Directory -Path $installRoot -Force | Out-Null
Copy-Item $launcherSource $launcherPath -Force

Write-Step 'Validating launcher syntax with Windows PowerShell...'
$tokens = $null
$parseErrors = $null
[System.Management.Automation.Language.Parser]::ParseFile(
    $launcherPath,
    [ref]$tokens,
    [ref]$parseErrors
) | Out-Null
if ($parseErrors -and $parseErrors.Count -gt 0) {
    $messages = ($parseErrors | ForEach-Object { $_.Message }) -join '; '
    throw "Installed terminal launcher has PowerShell syntax errors: $messages"
}

$protocolRoot = 'HKCU:\Software\Classes\swing-terminal'
$commandKey = Join-Path $protocolRoot 'shell\open\command'
New-Item -Path $commandKey -Force | Out-Null
Set-Item -Path $protocolRoot -Value 'URL:SWING PI Terminal'
New-ItemProperty -Path $protocolRoot -Name 'URL Protocol' -Value '' -PropertyType String -Force | Out-Null

$powershell = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
$command = '"{0}" -NoProfile -ExecutionPolicy Bypass -File "{1}" "%1"' -f $powershell, $launcherPath
Set-Item -Path $commandKey -Value $command

$registered = (Get-Item -Path $commandKey).GetValue('')
if ([string]::IsNullOrWhiteSpace([string]$registered)) {
    throw 'Failed to register swing-terminal:// protocol.'
}

if (-not $Quiet) {
    Write-Host ''
    Write-Host 'SWING PI terminal protocol installed.' -ForegroundColor Green
    Write-Host '  Protocol : swing-terminal://'
    Write-Host "  Launcher : $launcherPath"
    Write-Host '  Action   : Windows Terminal -> CMD /k -> ssh'
    Write-Host '  Syntax   : validated'
    Write-Host ''
}
