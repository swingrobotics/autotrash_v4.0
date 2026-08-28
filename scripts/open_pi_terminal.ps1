param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$TargetUri
)

$ErrorActionPreference = 'Stop'

function Test-PrivateAddress([string]$HostName) {
    $address = $null
    if ([System.Net.IPAddress]::TryParse($HostName, [ref]$address)) {
        if ($address.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork) {
            $bytes = $address.GetAddressBytes()
            if ($bytes[0] -eq 10) { return $true }
            if ($bytes[0] -eq 127) { return $true }
            if ($bytes[0] -eq 169 -and $bytes[1] -eq 254) { return $true }
            if ($bytes[0] -eq 172 -and $bytes[1] -ge 16 -and $bytes[1] -le 31) { return $true }
            if ($bytes[0] -eq 192 -and $bytes[1] -eq 168) { return $true }
            return $false
        }

        if ($address.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetworkV6) {
            if ([System.Net.IPAddress]::IsLoopback($address)) { return $true }
            $bytes = $address.GetAddressBytes()
            if (($bytes[0] -band 0xFE) -eq 0xFC) { return $true }
            if ($bytes[0] -eq 0xFE -and (($bytes[1] -band 0xC0) -eq 0x80)) { return $true }
            return $false
        }
        return $false
    }

    if ($HostName -match '^[A-Za-z0-9][A-Za-z0-9.-]{0,252}$') {
        if ($HostName.EndsWith('.local', [System.StringComparison]::OrdinalIgnoreCase)) { return $true }
        if (-not $HostName.Contains('.')) { return $true }
    }
    return $false
}

function Show-TerminalError([string]$Message) {
    try {
        Add-Type -AssemblyName PresentationFramework -ErrorAction Stop
        [System.Windows.MessageBox]::Show($Message, 'SWING PI Terminal') | Out-Null
    } catch {
        Write-Host $Message -ForegroundColor Red
    }
}

try {
    $uri = New-Object System.Uri($TargetUri)
} catch {
    Show-TerminalError ("Invalid SWING terminal URI: " + $TargetUri)
    exit 2
}

if ($uri.Scheme -ne 'swing-terminal') {
    Show-TerminalError ("Unsupported terminal URI scheme: " + $uri.Scheme)
    exit 2
}

$user = [System.Uri]::UnescapeDataString([string]$uri.UserInfo)
if ([string]::IsNullOrWhiteSpace($user)) { $user = 'gnss' }
if ($user -notmatch '^[A-Za-z0-9._-]{1,32}$') {
    Show-TerminalError 'Invalid SSH user name.'
    exit 2
}

$hostName = [string]$uri.Host
if ([string]::IsNullOrWhiteSpace($hostName) -or -not (Test-PrivateAddress $hostName)) {
    Show-TerminalError ("Only private/local rover addresses are allowed: " + $hostName)
    exit 2
}

$destination = $user + '@' + $hostName
$ssh = Get-Command ssh.exe -ErrorAction SilentlyContinue
if (-not $ssh) {
    $ssh = Get-Command ssh -ErrorAction SilentlyContinue
}
if (-not $ssh) {
    Show-TerminalError 'Windows OpenSSH Client (ssh.exe) was not found. Install the OpenSSH Client optional feature.'
    exit 2
}

$cmd = $env:ComSpec
if ([string]::IsNullOrWhiteSpace($cmd) -or -not (Test-Path $cmd)) {
    $cmd = Join-Path $env:SystemRoot 'System32\cmd.exe'
}

$sshCommand = 'ssh -o ConnectTimeout=8 ' + $destination
$wt = Get-Command wt.exe -ErrorAction SilentlyContinue
if ($wt) {
    try {
        Start-Process -FilePath $wt.Source -ArgumentList @(
            'new-tab',
            $cmd,
            '/k',
            $sshCommand
        )
        exit 0
    } catch {
    }
}

Start-Process -FilePath $cmd -ArgumentList @('/k', $sshCommand)
