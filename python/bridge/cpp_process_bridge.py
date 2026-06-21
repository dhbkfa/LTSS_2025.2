"""
cpp_process_bridge.py - Cau noi goi C++ simulator binary tu Python.

Theo dung co che giao tiep da thong nhat: C++ ghi file JSON moi buoc,
Python doc file (don gian, de debug). Module nay chi lo viec KHOI CHAY
va THEO DOI tien trinh C++ chay nen; viec doc/hien thi JSON nam o run_viewer.py.
"""

import json
import os
import subprocess
import threading
import time


def find_simulator_binary(project_root):
    """Tim file thuc thi simulator da build san, theo vai vi tri thuong gap."""
    candidates = [
        os.path.join(project_root, "build", "simulator"),
        os.path.join(project_root, "build", "Release", "simulator.exe"),
        os.path.join(project_root, "build", "Debug", "simulator.exe"),
        os.path.join(project_root, "build", "simulator.exe"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


class SimulatorRun:
    """Dai dien cho mot lan chay simulator C++ nen, voi thu muc output rieng."""

    def __init__(self, project_root, map_path, output_dir,
                 robot_radius=0, safety_margin=0, max_timesteps=200000):
        self.project_root = project_root
        self.map_path = map_path
        self.output_dir = output_dir
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.max_timesteps = max_timesteps

        self.process = None
        self.thread = None
        self.stdout_lines = []
        self.stderr_lines = []
        self.finished = False
        self.return_code = None
        self.error_message = None

    def start(self):
        binary = find_simulator_binary(self.project_root)
        if binary is None:
            self.error_message = (
                "Khong tim thay file thuc thi 'simulator'. Hay build C++ truoc:\n"
                "  mkdir -p build && cd build && cmake .. && make"
            )
            self.finished = True
            return False

        os.makedirs(self.output_dir, exist_ok=True)
        # Don sach output cu de tranh doc nham step cua lan chay truoc
        for fn in os.listdir(self.output_dir):
            if fn.startswith("step_") or fn == "meta.json":
                try:
                    os.remove(os.path.join(self.output_dir, fn))
                except OSError:
                    pass

        cmd = [
            binary, self.map_path, self.output_dir,
            str(self.robot_radius), str(self.safety_margin), str(self.max_timesteps),
        ]

        try:
            self.process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )
        except OSError as e:
            self.error_message = f"Khong the khoi chay simulator: {e}"
            self.finished = True
            return False

        self.thread = threading.Thread(target=self._reader_thread, daemon=True)
        self.thread.start()
        return True

    def _reader_thread(self):
        out, err = self.process.communicate()
        self.stdout_lines = out.splitlines() if out else []
        self.stderr_lines = err.splitlines() if err else []
        self.return_code = self.process.returncode
        if self.return_code != 0 and not self.error_message:
            self.error_message = "\n".join(self.stderr_lines) or "Simulator ket thuc voi loi khong xac dinh."
        self.finished = True

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()

    def list_available_steps(self):
        """Tra ve danh sach so thu tu cac step da duoc C++ ghi ra (sap xep tang dan)."""
        if not os.path.isdir(self.output_dir):
            return []
        steps = []
        for fn in os.listdir(self.output_dir):
            if fn.startswith("step_") and fn.endswith(".json"):
                try:
                    steps.append(int(fn[5:11]))
                except ValueError:
                    continue
        return sorted(steps)

    def read_step(self, step_index):
        path = os.path.join(self.output_dir, f"step_{step_index:06d}.json")
        if not os.path.isfile(path):
            return None
        # File co the dang duoc C++ ghi do (race condition nhe) - thu lai vai lan ngan.
        for _ in range(5):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except (json.JSONDecodeError, OSError):
                time.sleep(0.01)
        return None

    def read_meta(self):
        path = os.path.join(self.output_dir, "meta.json")
        if not os.path.isfile(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError):
            return None
