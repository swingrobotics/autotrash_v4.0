# SWING_CAR

SWING_CAR is the rover autonomy, recording, model-training and Windows compute stack for the SWING vehicle platform.

`main` is the canonical integration branch. The Raspberry Pi remains the vehicle control and safety authority; the Windows **SWING Compute Worker** is an optional compute/training node.

> Autonomous software, model training and offline validation do not by themselves approve unattended or normal-speed vehicle operation. Use closed-area, low-speed validation and preserve the rover-side safety path.

## What runs where

| Component | Role | Main entrypoint |
|---|---|---|
| Raspberry Pi rover | Sensors, motor/steering control, E-STOP, watchdogs, RECORD, runtime autonomy | `server_v2_final.py` |
| Windows Compute Worker | RECORD transfer, model training/export, RECORD model preview, optional remote UFLD compute | `scripts/run_compute_worker.py` |
| Windows Compute Manager | Starts/stops the packaged Worker and opens status/data/log tools | `scripts/run_compute_manager.py` |
| Arduino | Low-level motor/steering serial control and watchdog | `arduino/motor_serial/motor_serial.ino` |

The final rover service is intentionally layered rather than duplicated:

```text
server_v2_final.py
  -> server_v2_release.py
    -> server_v2_full.py
      -> server_v2_ai.py
        -> server_v2.py
          -> server.py
```

See [docs/architecture.md](docs/architecture.md) before removing or merging those files.

## User-facing drive modes

The canonical modes are:

- `MANUAL` — human driving with hard safety.
- `RECORD` — human driving plus synchronized camera/sensor/control recording.
- `AUTO_AI` — route-independent learned driving.
- `AUTO_GPS` — normalized-route-conditioned learned driving.
- `AUTO_LOCAL` — saved local-map navigation using LiDAR/IMU localization and planning.
- `AUTO` — conservative strategy selector.

`DISARMED`, `EMERGENCY_STOP` and `FAULT` are system states. `PRETRAINED_ROAD` is an internal `AUTO` strategy, not an additional user-selectable mode.

## Windows Compute Worker

Normal Windows users should install the packaged release instead of Python dependencies.

1. Open this repository's **Releases** page.
2. Download `SWING-Compute-Worker-Setup.exe`.
3. Optionally verify `SWING-Compute-Worker-Setup.exe.sha256`.
4. Install and open **SWING Compute Worker**.
5. Start/stop the Worker from the Manager app.

The packaged Manager intentionally does **not** auto-start the Worker after a Windows reboot. The vehicle never transfers final motor/steering safety authority to the PC.

For source/release build details, see [docs/worker.md](docs/worker.md).

## AUTO_GPS temporal steering contract

New temporal AUTO_GPS models use recent **encoder-measured steering angle** as temporal vehicle state. Human target steering remains the supervised command label; the model's own previous prediction is not fed back as measured steering state for new v3 models.

The current RECORD format already keeps both values:

- `steering_angle_degrees` — actual encoder-measured steering.
- `target_steering_angle_degrees` — requested/human target steering.

See [docs/ai/AUTO_GPS.md](docs/ai/AUTO_GPS.md) for the training/runtime contract.

## Public repository data policy

Do not commit vehicle-generated or site-specific operational data to this repository. Keep these local or in explicitly access-controlled storage:

- RECORD sessions, camera/video captures and MCAP/log exports
- GPS/GNSS traces or field-test CSV files containing latitude/longitude
- normalized routes, saved maps and trained model artifacts
- NTRIP caster credentials, Wi-Fi passwords, API tokens and `.env` files
- SSH/private keys, certificates containing private keys, or local credential files
- vehicle-specific calibration/state files unless deliberately sanitized for publication

The `.gitignore` excludes the normal runtime locations and common credential/key patterns. Before publishing a new sample dataset or field log, remove exact location, network and identity metadata rather than relying only on `.gitignore`.

See [SECURITY.md](SECURITY.md) for reporting and secret-handling guidance.

## Documentation

Current documents:

- [Architecture](docs/architecture.md)
- [Windows Compute Worker](docs/worker.md)
- [AUTO_AI](docs/ai/AUTO_AI.md)
- [AUTO_GPS](docs/ai/AUTO_GPS.md)
- [AUTO_LOCAL](docs/ai/AUTO_LOCAL.md)
- [Closed-area validation](docs/validation/FIELD_TEST.md)
- [Third-party models and licenses](THIRD_PARTY_MODELS.md)

Historical planning/audit documents live under [docs/archive](docs/archive/README.md) and are not the current source of truth.

## Repository layout

```text
autonomous_car/       autonomy, AI, recording, localization, safety, validation
camera_stream/         extracted Raspberry Pi hardware helpers
swing_compute/         Windows Worker, training and preview pipelines
arduino/               motor/steering firmware
scripts/               operator, training, validation and build commands
packaging/windows/     Inno Setup installer definition
third_party/           vendored attributed adapters/licenses
docs/                  current and archived documentation
lidar-stability/       non-location LiDAR stability evidence
.github/workflows/     CI and Windows release automation
```

Generated runtime/model data such as `recordings/`, `field-tests/`, `models/`, `datasets/`, `gps-routes/`, `maps/`, `dist/` and `installer-dist/` are intentionally excluded by `.gitignore`.

## Licensing

There is currently **no project-wide open-source license grant** for SWING_CAR. Public visibility alone does not grant permission to copy, modify, redistribute or commercially use the project code beyond rights provided by applicable law.

Third-party components retain their own licenses and notices. See [THIRD_PARTY_MODELS.md](THIRD_PARTY_MODELS.md) and the license/notice files under `third_party/`.

## Development validation

Core rover regression:

```bash
python3 -m autonomous_car.simulation.validate_autonomy_v2
```

Temporal AUTO_GPS contract:

```bash
python3 -m autonomous_car.simulation.validate_temporal_gps_training_v2
```

RECORD model preview smoke:

```bash
python3 -m autonomous_car.simulation.validate_record_model_preview_v2
```

Windows release launcher from a checked-out `main` source tree:

```powershell
.\Release-SWING-Compute-Worker.cmd
```

The launcher validates source contracts, builds the Worker/Manager executables, produces the Inno Setup installer under `installer-dist/` and then uses GitHub CLI authentication to create/update the matching GitHub Release. Run `scripts/build_compute_worker_release.ps1` without `-Publish` when you only want a local build.
