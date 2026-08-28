"""Windows desktop manager for the SWING Compute Worker.

The manager is intentionally separate from the worker process. Installing the
Windows package does not add a Startup entry. The operator explicitly starts
and stops the worker from this application.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen
import webbrowser

import tkinter as tk
from tkinter import messagebox, ttk


MANAGER_VERSION = "0.3.0"
WORKER_HOST = "127.0.0.1"
WORKER_PORT = 8765
WORKER_BASE_URL = f"http://{WORKER_HOST}:{WORKER_PORT}"
WORKER_IMAGE_NAME = "SWING-Compute-Worker.exe"


def _is_frozen():
    return bool(getattr(sys, "frozen", False))


def _install_root():
    if _is_frozen():
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parents[1]


def _data_root():
    base = os.environ.get("LOCALAPPDATA") or os.environ.get("APPDATA")
    if base:
        return Path(base) / "SWING Robotics" / "Compute Worker"
    return Path.home() / ".swing-compute-worker"


def _worker_command():
    root = _install_root()
    packaged = root / "worker" / WORKER_IMAGE_NAME
    if packaged.is_file():
        return [str(packaged), "--background"], packaged.parent

    runner = root / "scripts" / "run_compute_worker.py"
    if runner.is_file():
        return [sys.executable, str(runner), "--background"], root
    raise FileNotFoundError("SWING Compute Worker executable was not found.")


def _http_json(path, timeout=1.5):
    request = Request(
        WORKER_BASE_URL + path,
        headers={"Accept": "application/json", "Cache-Control": "no-store"},
        method="GET",
    )
    with urlopen(request, timeout=timeout) as response:
        value = json.loads(response.read(1024 * 1024).decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("Worker returned an invalid response.")
    return value


def worker_healthy():
    try:
        return _http_json("/api/v1/health", timeout=0.8).get("service") == "swing-compute-worker"
    except Exception:
        return False


def _creation_flags():
    return int(getattr(subprocess, "CREATE_NO_WINDOW", 0))


def start_worker():
    if worker_healthy():
        return
    command, cwd = _worker_command()
    data_root = _data_root()
    logs_root = data_root / "logs"
    logs_root.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["SWING_COMPUTE_DATA_ROOT"] = str(data_root)
    with open(logs_root / "worker-manager.log", "ab", buffering=0) as log_file:
        subprocess.Popen(
            command,
            cwd=str(cwd),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log_file,
            stderr=log_file,
            creationflags=_creation_flags(),
            close_fds=True,
        )


def stop_worker():
    if os.name != "nt":
        raise RuntimeError("Stopping the packaged Worker is supported on Windows only.")

    helper = _install_root() / "tools" / "stop_swing_worker.ps1"
    if helper.is_file():
        subprocess.run(
            [
                "powershell.exe",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(helper),
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=_creation_flags(),
            check=False,
        )
        return

    subprocess.run(
        ["taskkill", "/IM", WORKER_IMAGE_NAME, "/T", "/F"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=_creation_flags(),
        check=False,
    )


def _format_gib(value):
    try:
        return f"{float(value) / (1024 ** 3):.1f} GB"
    except (TypeError, ValueError):
        return "-"


class ComputeManagerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SWING Compute Worker")
        self.root.geometry("720x500")
        self.root.minsize(650, 450)
        self._refresh_running = False
        self._closing = False

        self.state_var = tk.StringVar(value="확인 중")
        self.version_var = tk.StringVar(value="-")
        self.pc_var = tk.StringVar(value="-")
        self.compute_var = tk.StringVar(value="-")
        self.memory_var = tk.StringVar(value="-")
        self.data_var = tk.StringVar(value=str(_data_root()))
        self.detail_var = tk.StringVar(value="Worker 상태를 확인하고 있습니다.")

        self._build_ui()
        self.root.after(100, self.refresh_async)
        self.root.after(2500, self._periodic_refresh)

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="SWING Compute Worker", font=("Segoe UI", 18, "bold")).pack(anchor="w")
        ttk.Label(
            outer,
            text=f"Windows Manager v{MANAGER_VERSION} · 부팅 시 자동 실행하지 않습니다.",
            font=("Segoe UI", 9),
        ).pack(anchor="w", pady=(2, 14))

        status = ttk.LabelFrame(outer, text="상태", padding=12)
        status.pack(fill="x")
        grid = ttk.Frame(status)
        grid.pack(fill="x")
        fields = [
            ("WORKER", self.state_var),
            ("VERSION", self.version_var),
            ("PC", self.pc_var),
            ("AI COMPUTE", self.compute_var),
            ("MEMORY", self.memory_var),
        ]
        for index, (label, variable) in enumerate(fields):
            box = ttk.Frame(grid, padding=(6, 4))
            box.grid(row=0, column=index, sticky="nsew")
            grid.columnconfigure(index, weight=1)
            ttk.Label(box, text=label, font=("Segoe UI", 8)).pack(anchor="w")
            ttk.Label(box, textvariable=variable, font=("Consolas", 10, "bold")).pack(anchor="w", pady=(3, 0))
        ttk.Label(status, textvariable=self.detail_var, wraplength=640, justify="left").pack(anchor="w", pady=(10, 0))

        controls = ttk.LabelFrame(outer, text="Worker 제어", padding=12)
        controls.pack(fill="x", pady=(12, 0))
        row = ttk.Frame(controls)
        row.pack(fill="x")
        ttk.Button(row, text="Worker 시작", command=self.start_clicked).pack(side="left")
        ttk.Button(row, text="Worker 중지", command=self.stop_clicked).pack(side="left", padx=(8, 0))
        ttk.Button(row, text="새로고침", command=self.refresh_async).pack(side="left", padx=(8, 0))
        ttk.Button(row, text="상태 페이지 열기", command=self.open_status).pack(side="left", padx=(8, 0))

        storage = ttk.LabelFrame(outer, text="파일 / 관리", padding=12)
        storage.pack(fill="x", pady=(12, 0))
        ttk.Label(storage, text="데이터 폴더").pack(anchor="w")
        ttk.Label(storage, textvariable=self.data_var, font=("Consolas", 9), wraplength=640).pack(anchor="w", pady=(2, 8))
        row2 = ttk.Frame(storage)
        row2.pack(fill="x")
        ttk.Button(row2, text="데이터 폴더 열기", command=self.open_data).pack(side="left")
        ttk.Button(row2, text="로그 열기", command=self.open_logs).pack(side="left", padx=(8, 0))
        ttk.Button(row2, text="학습 데이터 삭제", command=self.delete_data).pack(side="left", padx=(8, 0))
        ttk.Button(row2, text="앱 제거", command=self.uninstall_app).pack(side="right")

        ttk.Label(
            outer,
            text="앱 창을 닫아도 실행 중인 Worker는 계속 동작합니다. Windows를 재부팅하면 Worker는 자동으로 시작되지 않습니다.",
            wraplength=650,
        ).pack(anchor="w", pady=(14, 0))

    def _periodic_refresh(self):
        if self._closing:
            return
        self.refresh_async()
        self.root.after(2500, self._periodic_refresh)

    def refresh_async(self):
        if self._refresh_running or self._closing:
            return
        self._refresh_running = True

        def fetch_status():
            try:
                status, error = _http_json("/api/v1/status", timeout=2.5), None
            except Exception as exc:
                status, error = None, exc
            self.root.after(0, lambda: self._apply_status(status, error))

        threading.Thread(target=fetch_status, daemon=True).start()

    def _apply_status(self, status, error):
        self._refresh_running = False
        if status is None:
            self.state_var.set("중지됨")
            self.version_var.set("-")
            self.pc_var.set("-")
            self.compute_var.set("-")
            self.memory_var.set("-")
            self.detail_var.set("Worker가 실행 중이 아닙니다.")
            return

        capabilities = status.get("capabilities") or {}
        self.state_var.set("연결됨")
        self.version_var.set(str(status.get("version") or "-"))
        self.pc_var.set(str(status.get("hostname") or "-"))
        self.memory_var.set(_format_gib((status.get("memory") or {}).get("total_bytes")))
        if capabilities.get("cuda"):
            compute = str(capabilities.get("gpu_name") or "CUDA GPU")
        elif capabilities.get("local_cpu_training"):
            compute = "CPU 학습"
        else:
            compute = "상태 확인만"
        self.compute_var.set(compute)
        self.data_var.set(str(status.get("data_root") or _data_root()))
        features = []
        if capabilities.get("openvino"):
            features.append("OpenVINO")
        if capabilities.get("onnx_runtime"):
            features.append("ONNX Runtime")
        if capabilities.get("record_cache"):
            features.append("RECORD cache")
        free = _format_gib((status.get("disk") or {}).get("free_bytes"))
        self.detail_var.set(f"{' · '.join(features) if features else '기본 Worker'} · 저장공간 {free} 남음")

    def start_clicked(self):
        try:
            start_worker()
            self.detail_var.set("Worker를 시작했습니다. 연결 확인 중...")
            self.root.after(700, self.refresh_async)
        except Exception as exc:
            messagebox.showerror("SWING Compute Worker", f"Worker 시작 실패:\n{exc}")

    def stop_clicked(self):
        try:
            if worker_healthy():
                stop_worker()
            self.detail_var.set("Worker를 중지했습니다.")
            self.root.after(600, self.refresh_async)
        except Exception as exc:
            messagebox.showerror("SWING Compute Worker", f"Worker 중지 실패:\n{exc}")

    @staticmethod
    def open_status():
        webbrowser.open(WORKER_BASE_URL + "/")

    @staticmethod
    def _open_path(path):
        if os.name == "nt":
            os.startfile(str(path))
        else:
            webbrowser.open(path.as_uri())

    def open_data(self):
        path = _data_root()
        path.mkdir(parents=True, exist_ok=True)
        self._open_path(path)

    def open_logs(self):
        path = _data_root() / "logs"
        path.mkdir(parents=True, exist_ok=True)
        self._open_path(path)

    def delete_data(self):
        path = _data_root()
        if not path.exists():
            messagebox.showinfo("SWING Compute Worker", "삭제할 학습 데이터가 없습니다.")
            return
        if not messagebox.askyesno(
            "학습 데이터 삭제",
            f"RECORD 캐시, 학습 작업, 모델/로그를 삭제합니다.\n\n{path}\n\n이 작업은 되돌릴 수 없습니다. 계속할까요?",
            icon="warning",
        ):
            return
        try:
            if worker_healthy():
                stop_worker()
                time.sleep(0.8)
            shutil.rmtree(path)
            self.detail_var.set("PC Worker 학습 데이터를 삭제했습니다.")
            self.refresh_async()
        except Exception as exc:
            messagebox.showerror("SWING Compute Worker", f"데이터 삭제 실패:\n{exc}")

    def uninstall_app(self):
        uninstaller = _install_root() / "unins000.exe"
        if not uninstaller.is_file():
            messagebox.showerror("SWING Compute Worker", "설치 프로그램의 제거 파일을 찾지 못했습니다.")
            return
        if not messagebox.askyesno(
            "앱 제거",
            "SWING Compute Worker 앱을 Windows에서 제거합니다.\n학습 데이터는 자동으로 삭제하지 않습니다.\n\n계속할까요?",
            icon="warning",
        ):
            return
        try:
            if worker_healthy():
                stop_worker()
                time.sleep(0.5)
            subprocess.Popen([str(uninstaller)], cwd=str(_install_root()))
            self._closing = True
            self.root.after(150, self.root.destroy)
        except Exception as exc:
            messagebox.showerror("SWING Compute Worker", f"앱 제거 시작 실패:\n{exc}")


def main():
    root = tk.Tk()
    try:
        style = ttk.Style(root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
    except tk.TclError:
        pass
    ComputeManagerApp(root)
    root.mainloop()
    return 0


__all__ = ["ComputeManagerApp", "MANAGER_VERSION", "main", "start_worker", "stop_worker", "worker_healthy"]
