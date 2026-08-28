$ErrorActionPreference = "Stop"

$resultPath = Join-Path $PSScriptRoot "ics-result.txt"

try {
    Set-Service -Name NlaSvc -StartupType Automatic
    Start-Service -Name NlaSvc
    Start-Service -Name Netman
    Start-Service -Name SharedAccess
    Start-Sleep -Seconds 2

    $manager = New-Object -ComObject HNetCfg.HNetShare
    $connections = @(
        $manager.EnumEveryConnection | Where-Object { $null -ne $_ }
    )
    if ($connections.Count -eq 0) {
        throw "Windows ICS returned no network connections"
    }

    $entries = foreach ($connection in $connections) {
        $properties = $manager.NetConnectionProps($connection)
        $configuration = $manager.INetSharingConfigurationForINetConnection($connection)
        [pscustomobject]@{
            Connection = $connection
            Properties = $properties
            Configuration = $configuration
        }
    }

    $public = $entries | Where-Object {
        $_.Properties.Name -eq "Wi-Fi" -or
        $_.Properties.DeviceName -like "*Wireless-AC 9462*"
    } | Select-Object -First 1
    $private = $entries | Where-Object {
        $_.Properties.DeviceName -like "*Realtek PCIe GbE*"
    } | Select-Object -First 1

    if ($null -eq $public) {
        throw "Wi-Fi internet adapter was not found"
    }
    if ($null -eq $private) {
        throw "Raspberry Pi Ethernet adapter was not found"
    }

    foreach ($entry in $entries) {
        if ($entry.Configuration.SharingEnabled) {
            $entry.Configuration.DisableSharing()
        }
    }
    Start-Sleep -Seconds 2

    $public.Configuration.EnableSharing(0)
    $private.Configuration.EnableSharing(1)
    Start-Sleep -Seconds 4

    $result = @(
        "ICS configured successfully"
        "Public: $($public.Properties.Name) enabled=$($public.Configuration.SharingEnabled) type=$($public.Configuration.SharingConnectionType)"
        "Private: $($private.Properties.Name) enabled=$($private.Configuration.SharingEnabled) type=$($private.Configuration.SharingConnectionType)"
        "NlaSvc: $((Get-Service NlaSvc).Status)"
        "SharedAccess: $((Get-Service SharedAccess).Status)"
        "Ethernet IPv4: $((Get-NetIPAddress -InterfaceAlias $private.Properties.Name -AddressFamily IPv4 | Select-Object -ExpandProperty IPAddress) -join ', ')"
    )
    $result | Set-Content -LiteralPath $resultPath -Encoding utf8
} catch {
    @(
        "ICS configuration failed"
        $_.Exception.Message
        $_.ScriptStackTrace
    ) | Set-Content -LiteralPath $resultPath -Encoding utf8
    exit 1
}
