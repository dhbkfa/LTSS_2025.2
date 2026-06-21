"""
main.py - Ung dung chinh: ghep tab 'Xay dung map' va tab 'Chay mo phong'.

Chay: python3 main.py
(can python3-tk; xem README.md o thu muc goc de biet cach cai dat / build C++ truoc)
"""

import os
import sys
import tkinter as tk
from tkinter import ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from map_builder import MapBuilderFrame
from run_viewer import RunViewerFrame


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mo phong 5 robot kham pha ban do luoi chua biet - A* + Frontier")
        self.geometry("1000x800")
        self.minsize(820, 600)

        # project_root = thu muc goc cua du an (chua thu muc cpp/, python/, config/, build/)
        self.project_root = os.path.abspath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
        )

        notebook = ttk.Notebook(self)
        notebook.pack(fill=tk.BOTH, expand=True)

        self.builder_frame = MapBuilderFrame(notebook, on_map_exported=self._on_map_exported)
        self.run_frame = RunViewerFrame(notebook, self.project_root)

        notebook.add(self.builder_frame, text="1. Xay dung map")
        notebook.add(self.run_frame, text="2. Chay mo phong")

        self.notebook = notebook

    def _on_map_exported(self, path):
        """Khi xuat map xong o tab 1, tu dong dien duong dan vao tab 2 va chuyen qua xem."""
        self.run_frame.set_map_path(path)
        self.notebook.select(self.run_frame)


if __name__ == "__main__":
    app = App()
    app.mainloop()
