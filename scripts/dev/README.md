# Developer-only helpers

These tools are intentionally kept out of the repository root because normal operators should use the packaged Windows installer and the production rover service.

- `Install-SWING-Compute-Worker-DEV.cmd` — source/developer Worker installation helper.
- `preview_server.py` — local fake-sensor dashboard/layout preview. It does not control hardware.

Production Windows users should use `SWING-Compute-Worker-Setup.exe` from GitHub Releases. Production rover runtime is `server_v2_final.py` through `camera-stream.service`.
