import json
import os
import re
import shutil
import time
from pathlib import Path


class MapStoreError(ValueError):
    pass


class MapStore:
    """Filesystem-backed registry for reusable LOCAL/SLAM maps.

    The SLAM engine owns the grid map format. MapStore owns stable IDs,
    human names, quality metadata, mapping sessions, and named destinations.
    """

    METADATA_FILE = "metadata.json"

    def __init__(self, root_path):
        self.root_path = Path(root_path)

    def list_maps(self):
        if not self.root_path.exists():
            return []
        result = []
        for path in sorted(self.root_path.iterdir()):
            if not path.is_dir():
                continue
            metadata_path = path / self.METADATA_FILE
            if not metadata_path.exists():
                continue
            try:
                result.append(self._read_json(metadata_path))
            except (OSError, json.JSONDecodeError):
                continue
        return result

    def create_map(self, name, map_id=None, metadata=None):
        name = str(name or "").strip()
        if not name:
            raise MapStoreError("Map name is required")

        map_id = self._normalize_id(map_id or name)
        path = self.root_path / map_id
        if path.exists():
            raise MapStoreError(f"Map already exists: {map_id}")

        self.root_path.mkdir(parents=True, exist_ok=True)
        path.mkdir()

        now = time.time()
        document = {
            "map_id": map_id,
            "name": name,
            "created_at": now,
            "updated_at": now,
            "format": None,
            "map_file": None,
            "origin": None,
            "quality": {},
            "mapping_sessions": [],
            "destinations": [],
        }
        document.update(dict(metadata or {}))
        document["map_id"] = map_id
        document["name"] = name
        document["created_at"] = document.get("created_at", now)
        document["updated_at"] = now
        self._write_json(path / self.METADATA_FILE, document)
        return document

    def get_map(self, map_id):
        path = self.map_path(map_id) / self.METADATA_FILE
        if not path.exists():
            raise MapStoreError(f"Unknown map: {map_id}")
        return self._read_json(path)

    def update_map(self, map_id, **changes):
        path = self.map_path(map_id) / self.METADATA_FILE
        document = self.get_map(map_id)
        protected = {"map_id", "created_at"}
        for key, value in changes.items():
            if key not in protected:
                document[key] = value
        document["updated_at"] = time.time()
        self._write_json(path, document)
        return document

    def register_map_asset(self, map_id, filename, map_format, origin=None, quality=None):
        filename = os.path.basename(str(filename or ""))
        if not filename:
            raise MapStoreError("Map asset filename is required")
        return self.update_map(
            map_id,
            map_file=filename,
            format=str(map_format or ""),
            origin=origin,
            quality=dict(quality or {}),
        )

    def asset_path(self, map_id, filename=None):
        document = self.get_map(map_id)
        filename = os.path.basename(str(filename or document.get("map_file") or ""))
        if not filename:
            raise MapStoreError("Map asset is not registered")
        path = (self.map_path(map_id) / filename).resolve()
        root = self.map_path(map_id).resolve()
        if os.path.commonpath([str(root), str(path)]) != str(root):
            raise MapStoreError("Map asset path escapes map directory")
        return path

    def delete_map(self, map_id):
        path = self.map_path(map_id)
        if not path.exists():
            raise MapStoreError(f"Unknown map: {map_id}")
        shutil.rmtree(path)
        return {"deleted": True, "map_id": self._normalize_id(map_id)}

    def add_mapping_session(self, map_id, session_id):
        document = self.get_map(map_id)
        sessions = list(document.get("mapping_sessions") or [])
        session_id = str(session_id or "").strip()
        if not session_id:
            raise MapStoreError("Mapping session ID is required")
        if session_id not in sessions:
            sessions.append(session_id)
        return self.update_map(map_id, mapping_sessions=sessions)

    def upsert_destination(self, map_id, destination_id, name, x, y, heading_degrees=None):
        document = self.get_map(map_id)
        destination_id = self._normalize_id(destination_id or name)
        destination = {
            "destination_id": destination_id,
            "name": str(name or destination_id),
            "x": float(x),
            "y": float(y),
            "heading_degrees": (
                None if heading_degrees is None else float(heading_degrees)
            ),
        }
        destinations = [
            item
            for item in (document.get("destinations") or [])
            if item.get("destination_id") != destination_id
        ]
        destinations.append(destination)
        self.update_map(map_id, destinations=destinations)
        return destination

    def get_destination(self, map_id, destination_id):
        destination_id = self._normalize_id(destination_id)
        for item in self.get_map(map_id).get("destinations") or []:
            if item.get("destination_id") == destination_id:
                return item
        raise MapStoreError(f"Unknown destination: {destination_id}")

    def remove_destination(self, map_id, destination_id):
        document = self.get_map(map_id)
        destination_id = self._normalize_id(destination_id)
        destinations = [
            item
            for item in (document.get("destinations") or [])
            if item.get("destination_id") != destination_id
        ]
        return self.update_map(map_id, destinations=destinations)

    def map_path(self, map_id):
        path = self.root_path / self._normalize_id(map_id)
        root = self.root_path.resolve()
        resolved = path.resolve()
        if os.path.commonpath([str(root), str(resolved)]) != str(root):
            raise MapStoreError("Map path escapes map root")
        return path

    @staticmethod
    def _normalize_id(value):
        value = re.sub(r"[^a-zA-Z0-9._-]+", "_", str(value or "").strip())
        value = value.strip("._-").lower()
        if not value:
            raise MapStoreError("Map ID is required")
        return value

    @staticmethod
    def _read_json(path):
        with open(path, "r", encoding="utf-8") as file:
            return json.load(file)

    @staticmethod
    def _fsync_parent(path):
        parent = os.path.dirname(os.path.abspath(str(path))) or "."
        try:
            descriptor = os.open(parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        except OSError:
            return
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    @classmethod
    def _write_json(cls, path, document):
        temporary = Path(str(path) + ".tmp")
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2, sort_keys=True)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
        cls._fsync_parent(path)
