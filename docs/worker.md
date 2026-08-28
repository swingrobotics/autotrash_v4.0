# SWING Compute Worker

`SWING Compute Worker` is the Windows-side compute service for SWING_CAR. The rover remains the motor/steering and safety authority.

## Normal installation

Normal users should use the GitHub Release installer rather than installing Python packages manually.

Download:

```text
SWING-Compute-Worker-Setup.exe
SWING-Compute-Worker-Setup.exe.sha256
```

The installer places the packaged Worker under Program Files, installs the desktop/Start Menu Manager application and creates a private-network firewall rule for TCP port `8765`.

The current installer deliberately removes legacy Startup shortcuts. The Manager explicitly states that the Worker does **not** auto-start after Windows reboot.

Typical flow:

1. Install `SWING-Compute-Worker-Setup.exe`.
2. Open **SWING Compute Worker**.
3. Click **Worker 시작**.
4. Use the Manager status/data/log controls as needed.
5. Closing the Manager window does not automatically terminate an already-running Worker.

## Safety boundary

The Worker is not the final actuator controller. The Raspberry Pi retains:

- motor and steering control
- E-STOP
- watchdogs
- Arduino/steering health and limits
- required sensor freshness validation
- `SafetySupervisor`
- fail-closed stop/fault behavior

Loss of the Windows PC or Ethernet link must invalidate remote compute rather than bypass rover safety.

## Worker endpoints and storage

Default local status URL:

```text
http://127.0.0.1:8765/
```

Core endpoints include:

```text
GET /api/v1/health
GET /api/v1/status
POST /api/v1/jobs
```

Optional live UFLD endpoints are exposed under `/api/v1/perception/ufld`.

Managed runtime data is stored under:

```text
%LOCALAPPDATA%\SWING Robotics\Compute Worker
```

The Manager uses that location for Worker data and logs.

## Current compute scope

The Worker supports:

- diagnostic jobs
- RECORD synchronization/transfer
- AUTO_AI training/evaluation/export
- AUTO_GPS training/evaluation/export
- synchronized RECORD model preview
- H.264 preview artifact handling
- optional live UFLD compute
- runtime capability/CPU/RAM/disk reporting

RECORD model preview is diagnostic only and has no control authority.

## AUTO_GPS temporal training

New temporal AUTO_GPS candidates use encoder-measured steering history. Training target steering and measured steering are separate fields:

```text
target_steering_angle_degrees -> supervised desired command
steering_angle_degrees        -> actual measured vehicle state
```

The exported v3 contract requires measured steering feedback at runtime and forbids treating prior model prediction as the measured state.

## Source development

Install source dependencies:

```powershell
python -m pip install -r requirements-compute-worker.txt
```

Run:

```powershell
python scripts/run_compute_worker.py
```

For developer installation helpers, see `scripts/dev/`.

## Build the Windows installer locally

Build without publishing:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\build_compute_worker_release.ps1
```

The build script:

1. verifies Python/dependencies
2. compiles source
3. runs Worker/temporal GPS/RECORD preview contracts
4. builds Worker and Manager with PyInstaller
5. resolves/installs Inno Setup 6
6. builds `installer-dist\SWING-Compute-Worker-Setup.exe`
7. writes the SHA-256 sidecar

## Build and publish a GitHub Release

The root convenience launcher runs the same build with `-Publish`:

```powershell
.\Release-SWING-Compute-Worker.cmd
```

After a successful build it ensures GitHub CLI is available, requests browser authentication if needed, and creates or updates the matching `compute-worker-vX.Y.Z` release.

The repository workflow `.github/workflows/compute-worker-release.yml` provides the automated Windows build/release path from `main`.

## GitHub Release automation

The release workflow is `main`-based and uses `.github/releases/compute-worker-version.txt` or a manual workflow version input. It publishes:

```text
SWING-Compute-Worker-Setup.exe
SWING-Compute-Worker-Setup.exe.sha256
```

to a `compute-worker-vX.Y.Z` GitHub Release.

## Remaining physical validation

Before relying on remote compute for driving-related observations, measure the real vehicle Ethernet end-to-end latency and confirm failure behavior on the target PC/Pi combination. Source/CI success is not a substitute for that hardware validation.
