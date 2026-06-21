"""
map_builder.py - Giao dien xay dung ban do bang tay.

GIU NGUYEN tinh than code Python goc cua nguoi dung (Generate Maze preset,
ve active/inactive, ve/xoa tuong, sinh maze DFS) nhung sua cac diem chua hop
ly da neu trong bao cao thiet ke muc 3.1:
  - Thay vi 1 robot (rx, ry) va 1 goal -> ho tro dat TOI DA 5 VI TRI XUAT PHAT.
  - Khong con tu chay A* trong Python (A* gio thuoc ve C++ core).
  - Map sau khi ve xong duoc EXPORT RA JSON dung dinh dang TruthMap::loadFromJson
    ben C++ doc duoc, thay vi Python tu mo phong 1 robot.
"""

import json
import os
import random
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from common import C_INACTIVE, C_CELL, C_OCCUPIED, C_START, C_GOAL_BG, C_GRID_LINE, robot_color

DIR_X = [0, 1, 0, -1]   # N, E, S, W (khop quy uoc kDirX ben C++)
DIR_Y = [1, 0, -1, 0]
OPP = [2, 3, 0, 1]
DIR_NAMES = ["N", "E", "S", "W"]

MAX_ROBOTS = 5

PRESETS = {
    "Chu nhat (day du)": "full",
    "Bo goc tren-phai": "cut_tr",
    "Bo 4 goc": "cut_4c",
    "Hinh L": "shape_l",
    "Hinh T": "shape_t",
    "Hinh U": "shape_u",
    "Hinh chu thap (+)": "shape_plus",
}


class MapBuilderFrame(tk.Frame):
    """Frame xay dung map, nhung khong con chay thuat toan - chi xay va export."""

    def __init__(self, parent, on_map_exported=None):
        super().__init__(parent)
        self.on_map_exported = on_map_exported  # callback(path) khi export thanh cong, de Run tab tu load

        self.W, self.H = 16, 16

        # occupied[x][y] = True neu la vat can. active[x][y] = True neu o ton tai trong hinh dang map.
        self.occupied = [[False] * self.H for _ in range(self.W)]
        self.active = [[True] * self.H for _ in range(self.W)]
        # walls noi bo, rieng voi bien gioi map (bien gioi luon co tuong ngam dinh)
        self.walls = [[[False] * 4 for _ in range(self.H)] for _ in range(self.W)]

        self.start_positions = []   # list[(x,y)], toi da 5
        self.target = None          # (x,y) hoac None

        self.draw_mode = False       # che do ve active/inactive bang chuot keo
        self.draw_value = True
        self.place_mode = "start"    # "start" hoac "target" - quyet dinh click trai lam gi

        self._build_ui()
        self.after(100, self._init_default)

    # ---------------------------------------------------------------- UI ----

    def _build_ui(self):
        def mkbtn(parent, text, cmd, bg="#dddddd", fg="black"):
            b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg,
                           font=("Arial", 10), padx=5)
            b.pack(side=tk.LEFT, padx=2)
            return b

        # Row 1: kich thuoc + do mo + hinh dang preset
        r1 = tk.Frame(self, pady=3)
        r1.pack(side=tk.TOP, fill=tk.X, padx=6)

        tk.Label(r1, text="Rong(W):", font=("Arial", 10)).pack(side=tk.LEFT)
        self.w_var = tk.IntVar(value=16)
        tk.Spinbox(r1, from_=5, to=100, textvariable=self.w_var,
                   width=4, font=("Arial", 10)).pack(side=tk.LEFT, padx=(1, 6))

        tk.Label(r1, text="Cao(H):", font=("Arial", 10)).pack(side=tk.LEFT)
        self.h_var = tk.IntVar(value=16)
        tk.Spinbox(r1, from_=5, to=100, textvariable=self.h_var,
                   width=4, font=("Arial", 10)).pack(side=tk.LEFT, padx=(1, 8))

        tk.Label(r1, text="Do mo(%):", font=("Arial", 10)).pack(side=tk.LEFT)
        self.open_var = tk.IntVar(value=20)
        tk.Scale(r1, from_=0, to=80, orient=tk.HORIZONTAL,
                 variable=self.open_var, length=110,
                 showvalue=True, font=("Arial", 9)).pack(side=tk.LEFT, padx=(1, 8))

        tk.Label(r1, text="Hinh:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.preset_var = tk.StringVar(value="Chu nhat (day du)")
        cb = ttk.Combobox(r1, textvariable=self.preset_var,
                          values=list(PRESETS.keys()), width=18,
                          font=("Arial", 10), state="readonly")
        cb.pack(side=tk.LEFT, padx=(1, 0))
        cb.bind("<<ComboboxSelected>>", lambda _: self._apply_preset())

        # Row 2: cac nut hanh dong
        r2 = tk.Frame(self, pady=3)
        r2.pack(side=tk.TOP, fill=tk.X, padx=6)

        mkbtn(r2, "Tao map moi (kich thuoc)", self.reset_shape, "#4caf50", "white")
        mkbtn(r2, "Sinh ngau nhien (maze DFS)", self.generate_maze, "#2196f3", "white")

        self.draw_btn = tk.Button(r2, text="Ve vung: TAT", font=("Arial", 10),
                                  bg="#aaaaaa", fg="white", padx=5,
                                  command=self.toggle_draw)
        self.draw_btn.pack(side=tk.LEFT, padx=2)

        mkbtn(r2, "Xoa het robot", self.clear_starts)
        mkbtn(r2, "Xoa dich", self.clear_target)

        # Row 3: che do dat doi tuong (start / target) + export
        r3 = tk.Frame(self, pady=3)
        r3.pack(side=tk.TOP, fill=tk.X, padx=6)

        tk.Label(r3, text="Click trai de dat:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.place_var = tk.StringVar(value="start")
        tk.Radiobutton(r3, text=f"Vi tri robot (toi da {MAX_ROBOTS})", variable=self.place_var,
                       value="start", font=("Arial", 10),
                       command=lambda: setattr(self, "place_mode", "start")).pack(side=tk.LEFT)
        tk.Radiobutton(r3, text="Dich that", variable=self.place_var,
                       value="target", font=("Arial", 10),
                       command=lambda: setattr(self, "place_mode", "target")).pack(side=tk.LEFT, padx=(4, 12))

        tk.Label(r3, text="  (Click phai / keo chuot phai = dat vat can khi 've vung' TAT)",
                 font=("Arial", 8), fg="#666666").pack(side=tk.LEFT)

        # Row 4: export & load
        r4 = tk.Frame(self, pady=3)
        r4.pack(side=tk.TOP, fill=tk.X, padx=6)
        mkbtn(r4, "Xuat map ra JSON (cho C++)...", self.export_json, "#ff9800", "white")
        mkbtn(r4, "Tai map tu JSON...", self.load_json, "#9c27b0", "white")
        self.export_path_var = tk.StringVar(value="")
        tk.Label(r4, textvariable=self.export_path_var, font=("Arial", 9), fg="#006600").pack(side=tk.LEFT, padx=6)

        # Status bar
        r5 = tk.Frame(self)
        r5.pack(side=tk.TOP, fill=tk.X, padx=6)
        self.sv = tk.StringVar(value="Nhan 'Tao map moi' de bat dau.")
        tk.Label(r5, textvariable=self.sv, font=("Arial", 10),
                 fg="#333333", anchor="w").pack(fill=tk.X)

        # Legend
        r6 = tk.Frame(self)
        r6.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(0, 2))
        for color, label in [
            (C_START, "Vi tri robot"), (C_GOAL_BG, "Dich that"),
            (C_OCCUPIED, "Vat can"), (C_INACTIVE, "Vung khong ton tai"),
        ]:
            tk.Label(r6, bg=color, width=2).pack(side=tk.LEFT, padx=(4, 1))
            tk.Label(r6, text=label, font=("Arial", 9)).pack(side=tk.LEFT, padx=(0, 6))

        # Canvas
        self.canvas = tk.Canvas(self, bg="#aaaaaa", width=680, height=560)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        self.canvas.bind("<Button-1>", self._lclick)
        self.canvas.bind("<B1-Motion>", self._ldrag)
        self.canvas.bind("<Button-3>", self._rclick)
        self.canvas.bind("<B3-Motion>", self._rdrag)
        self.canvas.bind("<Configure>", lambda _: self._draw())

    # ---------------------------------------------------------- Helpers ----

    def _init_default(self):
        self.W = self.w_var.get()
        self.H = self.h_var.get()
        self._alloc_arrays()
        self._draw()

    def _alloc_arrays(self):
        self.active = [[True] * self.H for _ in range(self.W)]
        self.occupied = [[False] * self.H for _ in range(self.W)]
        self.walls = [[[False] * 4 for _ in range(self.H)] for _ in range(self.W)]
        self.start_positions = []
        self.target = None

    def _cs(self):
        cw = self.canvas.winfo_width() or 680
        ch = self.canvas.winfo_height() or 560
        return max(4, min(cw // max(1, self.W), ch // max(1, self.H)))

    def _p2c(self, px, py):
        cs = self._cs()
        return px // cs, self.H - 1 - py // cs

    def _valid(self, x, y):
        return 0 <= x < self.W and 0 <= y < self.H

    # ------------------------------------------------------- Mouse events ----

    def _wall_hit(self, px, py):
        cs = self._cs()
        lx = px % cs
        ly = py % cs
        thr = max(3, cs // 4)
        dists = [ly, cs - lx, cs - ly, lx]
        min_d = min(dists)
        return dists.index(min_d) if min_d < thr else None

    def _toggle_wall(self, x, y, d):
        W, H = self.W, self.H
        nx, ny = x + DIR_X[d], y + DIR_Y[d]
        if not (0 <= nx < W and 0 <= ny < H):
            return
        if not self.active[x][y] or not self.active[nx][ny]:
            return
        new_val = not self.walls[x][y][d]
        self.walls[x][y][d] = new_val
        self.walls[nx][ny][OPP[d]] = new_val
        action = "Them" if new_val else "Xoa"
        self.sv.set(f"{action} tuong tai ({x},{y}) huong {DIR_NAMES[d]}.")
        self._draw()

    def _lclick(self, e):
        x, y = self._p2c(e.x, e.y)
        if not self._valid(x, y):
            return
        if self.draw_mode:
            self.draw_value = not self.active[x][y]
            self.active[x][y] = self.draw_value
            if not self.draw_value:
                self.occupied[x][y] = False
                self._remove_start_at(x, y)
                if self.target == (x, y):
                    self.target = None
            self._draw()
            return

        wall_dir = self._wall_hit(e.x, e.y)
        if wall_dir is not None:
            self._toggle_wall(x, y, wall_dir)
            return

        if not self.active[x][y] or self.occupied[x][y]:
            self.sv.set("O nay khong hop le (khong active hoac la vat can).")
            return

        if self.place_mode == "start":
            self._toggle_start(x, y)
        else:
            self.target = (x, y)
            self.sv.set(f"Dich that dat tai ({x},{y}).")
            self._draw()

    def _toggle_start(self, x, y):
        pos = (x, y)
        if pos in self.start_positions:
            self.start_positions.remove(pos)
            self.sv.set(f"Da xoa vi tri robot tai {pos}.")
        else:
            if len(self.start_positions) >= MAX_ROBOTS:
                messagebox.showwarning("Qua so luong", f"Chi duoc dat toi da {MAX_ROBOTS} vi tri robot.")
                return
            self.start_positions.append(pos)
            self.sv.set(f"Da dat robot #{len(self.start_positions)-1} tai {pos}.")
        self._draw()

    def _remove_start_at(self, x, y):
        self.start_positions = [p for p in self.start_positions if p != (x, y)]

    def _ldrag(self, e):
        x, y = self._p2c(e.x, e.y)
        if self._valid(x, y) and self.draw_mode:
            self.active[x][y] = self.draw_value
            if not self.draw_value:
                self.occupied[x][y] = False
                self._remove_start_at(x, y)
                if self.target == (x, y):
                    self.target = None
            self._draw()

    def _rclick(self, e):
        x, y = self._p2c(e.x, e.y)
        if not self._valid(x, y):
            return
        if self.draw_mode:
            self.draw_value = False
            self.active[x][y] = False
            self.occupied[x][y] = False
            self._remove_start_at(x, y)
            if self.target == (x, y):
                self.target = None
            self._draw()
        else:
            if not self.active[x][y]:
                return
            if (x, y) in self.start_positions or self.target == (x, y):
                self.sv.set("Khong the dat vat can len o dang co robot/dich.")
                return
            self.occupied[x][y] = not self.occupied[x][y]
            self._draw()

    def _rdrag(self, e):
        x, y = self._p2c(e.x, e.y)
        if not self._valid(x, y):
            return
        if self.draw_mode:
            self.active[x][y] = False
            self.occupied[x][y] = False
            self._remove_start_at(x, y)
            if self.target == (x, y):
                self.target = None
            self._draw()
        elif self.active[x][y] and (x, y) not in self.start_positions and self.target != (x, y):
            self.occupied[x][y] = True
            self._draw()

    def toggle_draw(self):
        self.draw_mode = not self.draw_mode
        self.draw_btn.config(text=f"Ve vung: {'BAT' if self.draw_mode else 'TAT'}")

    # ------------------------------------------------------- Shape presets ----

    def _apply_preset(self):
        key = PRESETS[self.preset_var.get()]
        W, H = self.w_var.get(), self.h_var.get()
        act = [[True] * H for _ in range(W)]

        if key == "cut_tr":
            for x in range(W // 2, W):
                for y in range(H // 2, H):
                    act[x][y] = False
        elif key == "cut_4c":
            qw, qh = max(1, W // 4), max(1, H // 4)
            for x in range(W):
                for y in range(H):
                    if (x < qw or x >= W - qw) and (y < qh or y >= H - qh):
                        act[x][y] = False
        elif key == "shape_l":
            for x in range(W):
                for y in range(H):
                    if x >= W // 4 and y >= H // 4:
                        act[x][y] = False
        elif key == "shape_t":
            for x in range(W):
                for y in range(H):
                    in_bar = y >= H * 3 // 4
                    in_stem = W // 3 <= x < 2 * W // 3
                    if not in_bar and not in_stem:
                        act[x][y] = False
        elif key == "shape_u":
            for x in range(W):
                for y in range(H):
                    in_left = x < W // 4
                    in_right = x >= 3 * W // 4
                    in_bottom = y < H // 4
                    if not in_left and not in_right and not in_bottom:
                        act[x][y] = False
        elif key == "shape_plus":
            for x in range(W):
                for y in range(H):
                    in_h = W // 3 <= x < 2 * W // 3
                    in_v = H // 3 <= y < 2 * H // 3
                    if not in_h and not in_v:
                        act[x][y] = False

        self.W, self.H = W, H
        self.active = act
        self.occupied = [[False] * H for _ in range(W)]
        self.walls = [[[False] * 4 for _ in range(H)] for _ in range(W)]
        self.start_positions = []
        self.target = None
        self._draw()

    def reset_shape(self):
        self.W = self.w_var.get()
        self.H = self.h_var.get()
        self._alloc_arrays()
        self.sv.set(f"Da tao map moi {self.W}x{self.H}.")
        self._draw()

    # --------------------------------------------------- Sinh maze ngau nhien ----

    def generate_maze(self):
        """Sinh vat can ngau nhien bang DFS (giu tinh than maze generator goc),
        dam bao toan bo vung active deu lien thong (khong tao vung co lap),
        roi them mot ty le 'do mo' theo thanh truot de tao loop/duong tat."""
        W, H = self.W, self.H
        if not any(any(row) for row in self.active):
            messagebox.showwarning("Chua co map", "Hay tao map (chon kich thuoc/hinh dang) truoc.")
            return

        self.occupied = [[True] * H for _ in range(W)]
        self.walls = [[[False] * 4 for _ in range(H)] for _ in range(W)]

        start = None
        for x in range(W):
            for y in range(H):
                if self.active[x][y]:
                    start = (x, y)
                    break
            if start:
                break
        if start is None:
            messagebox.showwarning("Map rong", "Khong co o nao active.")
            return

        stack = [start]
        visited = set([start])
        self.occupied[start[0]][start[1]] = False

        while stack:
            cx, cy = stack[-1]
            neighbors = []
            dirs = list(range(4))
            random.shuffle(dirs)
            for d in dirs:
                nx, ny = cx + DIR_X[d], cy + DIR_Y[d]
                if 0 <= nx < W and 0 <= ny < H and self.active[nx][ny] and (nx, ny) not in visited:
                    neighbors.append((nx, ny))
            if neighbors:
                nx, ny = neighbors[0]
                self.occupied[nx][ny] = False
                visited.add((nx, ny))
                stack.append((nx, ny))
            else:
                stack.pop()

        open_pct = self.open_var.get() / 100.0
        for x in range(W):
            for y in range(H):
                if self.active[x][y] and self.occupied[x][y] and random.random() < open_pct:
                    self.occupied[x][y] = False

        self.start_positions = []
        self.target = None
        self.sv.set(f"Da sinh maze ngau nhien {W}x{H}. Hay dat vi tri robot va dich.")
        self._draw()

    def clear_starts(self):
        self.start_positions = []
        self._draw()

    def clear_target(self):
        self.target = None
        self._draw()

    # -------------------------------------------------------------- Draw ----

    def _draw(self):
        self.canvas.delete("all")
        cs = self._cs()
        for x in range(self.W):
            for y in range(self.H):
                x0 = x * cs
                y0 = (self.H - 1 - y) * cs
                x1, y1 = x0 + cs, y0 + cs

                if not self.active[x][y]:
                    color = C_INACTIVE
                elif self.occupied[x][y]:
                    color = C_OCCUPIED
                elif (x, y) == self.target:
                    color = C_GOAL_BG
                elif (x, y) in self.start_positions:
                    color = C_START
                else:
                    color = C_CELL

                self.canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline=C_GRID_LINE)

                if (x, y) in self.start_positions:
                    idx = self.start_positions.index((x, y))
                    self.canvas.create_text((x0 + x1) // 2, (y0 + y1) // 2, text=str(idx),
                                            fill=robot_color(idx), font=("Arial", max(8, cs // 2), "bold"))

                if self.active[x][y]:
                    if self.walls[x][y][0]:
                        self.canvas.create_line(x0, y0, x1, y0, fill="black", width=3)
                    if self.walls[x][y][1]:
                        self.canvas.create_line(x1, y0, x1, y1, fill="black", width=3)
                    if self.walls[x][y][2]:
                        self.canvas.create_line(x0, y1, x1, y1, fill="black", width=3)
                    if self.walls[x][y][3]:
                        self.canvas.create_line(x0, y0, x0, y1, fill="black", width=3)

        self.canvas.create_rectangle(2, 2, self.W * cs, self.H * cs, outline="black", width=2)

    # ------------------------------------------------------------ Export ----

    def to_json_dict(self):
        """Chuyen trang thai map hien tai sang dict dung dinh dang TruthMap::loadFromJson ben C++."""
        active = [[self.active[x][y] for y in range(self.H)] for x in range(self.W)]
        occupied = [[self.occupied[x][y] for y in range(self.H)] for x in range(self.W)]

        walls = {}
        dir_keys = ["N", "E", "S", "W"]
        for d, key in enumerate(dir_keys):
            walls[key] = [[self.walls[x][y][d] for y in range(self.H)] for x in range(self.W)]

        return {
            "width": self.W,
            "height": self.H,
            "active": active,
            "occupied": occupied,
            "walls": walls,
            "start_positions": [list(p) for p in self.start_positions],
            "target": list(self.target) if self.target else None,
        }

    def export_json(self):
        if len(self.start_positions) == 0:
            messagebox.showwarning("Chua co robot", "Hay dat it nhat 1 vi tri xuat phat cho robot truoc khi xuat.")
            return

        default_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "..", "config", "maps")
        default_dir = os.path.abspath(default_dir)
        os.makedirs(default_dir, exist_ok=True)

        path = filedialog.asksaveasfilename(
            title="Xuat map ra JSON",
            initialdir=default_dir,
            initialfile="map_custom.json",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
        )
        if not path:
            return

        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.to_json_dict(), f, indent=2)
            self.sv.set(f"Da xuat thanh cong: {os.path.basename(path)}")
            self.export_path_var.set(path)
            if self.on_map_exported:
                self.on_map_exported(path)
        except OSError as e:
            messagebox.showerror("Loi luu file", str(e))

    def load_json(self):
        """Tai map tu file JSON de xem va chinh sua."""
        default_dir = os.path.join(os.path.dirname(__file__), "..", "..", "config", "maps")
        path = filedialog.askopenfilename(
            title="Chon file map JSON de tai", initialdir=default_dir,
            filetypes=[("JSON files", "*.json")],
        )
        if not path:
            return
        
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            w = data.get("width", 16)
            h = data.get("height", 16)
            
            self.W, self.H = w, h
            self.w_var.set(w)
            self.h_var.set(h)
            
            # Khoi tao lai trang thai
            self.active = [[True] * h for _ in range(w)]
            self.occupied = [[False] * h for _ in range(w)]
            self.walls = [[[False] * 4 for _ in range(h)] for _ in range(w)]
            self.start_positions = []
            self.target = None
            
            if "active" in data:
                for x in range(w):
                    for y in range(h):
                        if x < len(data["active"]) and y < len(data["active"][x]):
                            self.active[x][y] = data["active"][x][y]
                            
            if "occupied" in data:
                for x in range(w):
                    for y in range(h):
                        if x < len(data["occupied"]) and y < len(data["occupied"][x]):
                            self.occupied[x][y] = data["occupied"][x][y]
                            
            if "walls" in data:
                dir_keys = ["N", "E", "S", "W"]
                for d, key in enumerate(dir_keys):
                    if key in data["walls"]:
                        wd = data["walls"][key]
                        for x in range(w):
                            for y in range(h):
                                if x < len(wd) and y < len(wd[x]):
                                    self.walls[x][y][d] = wd[x][y]
                                    
            if "start_positions" in data:
                self.start_positions = [(p[0], p[1]) for p in data["start_positions"]]
                
            if "target" in data and data["target"] is not None:
                self.target = (data["target"][0], data["target"][1])
                
            self.sv.set(f"Da tai map tu: {os.path.basename(path)}")
            self.export_path_var.set(path)
            self._draw()
            
        except Exception as e:
            messagebox.showerror("Loi tai map", f"Khong the doc file: {e}")
