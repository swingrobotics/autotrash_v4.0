# Camera stream hardware helpers

The production Raspberry Pi service is `server_v2_final.py`, launched by `camera-stream.service`.

`camera_stream/` contains hardware/helper modules that have been extracted from the large legacy backend without pretending the migration is complete.

Current modules:

- `config.py` — environment variables and hardware defaults.
- `camera.py` — camera capture helper.
- `lidar.py` — LD06 acquisition/scan state helper.
- `steering.py` — steering encoder calculations and safety limits.
- `motor.py` — motor-related helper surface used during extraction.

The full hardware/HTTP backend still depends on `server.py` through the layered V2 production service. Do not assume a missing `camera_stream/*.py` module exists based on an old extraction plan.

See `docs/architecture.md` for the current server composition.
