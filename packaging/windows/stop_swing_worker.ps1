$ErrorActionPreference = 'SilentlyContinue'

# Stop the process bound to the SWING Worker port only after verifying that the
# endpoint identifies itself as SWING Compute Worker. This avoids terminating an
# unrelated local service that happens to use another Python process.
$verified = $false
try {
    $health = Invoke-RestMethod -Uri 'http://127.0.0.1:8765/api/v1/health' -TimeoutSec 2
    $verified = ($health.service -eq 'swing-compute-worker')
} catch {}

if ($verified) {
    Get-NetTCPConnection -LocalPort 8765 -State Listen -ErrorAction SilentlyContinue |
        ForEach-Object {
            if ($_.OwningProcess -and $_.OwningProcess -ne $PID) {
                Stop-Process -Id $_.OwningProcess -Force -ErrorAction SilentlyContinue
            }
        }
}

# Also remove a stale developer worker whose HTTP listener may already be
# unhealthy. Match the command line and DEV install path together so unrelated
# Python processes are not touched.
Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
    Where-Object {
        $_.ProcessId -ne $PID -and
        $_.CommandLine -and
        $_.CommandLine -like '*run_compute_worker.py*' -and
        $_.CommandLine -like '*SWING Robotics*Compute Worker Dev*'
    } |
    ForEach-Object {
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }

Start-Sleep -Milliseconds 500
exit 0
