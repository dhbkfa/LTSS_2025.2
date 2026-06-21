"""
run_viewer.py - Giao dien CHAY va QUAN SAT mo phong 5 robot.

Theo dung kien truc muc 9 bao cao: Python CHI doc trang thai tu C++ qua JSON
va ve lai - khong tu chay thuat toan, khong biet ban do that. Vung chua duoc
robot kham pha (UNKNOWN) duoc ve mau toi (man suong chien tranh), dung yeu
cau "robot chi nen thay phan da kham pha" trong bao cao muc 3.1.

Luong du lieu: C++ simulation core -> JSON state (moi file 1 timestep) -> Python UI doc va ve.
"""

import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # cho phep import bridge

from common import CELL_STATE_COLOR, robot_color, PHASE_LABEL_VI, STATUS_LABEL_VI, C_GRID_LINE
from bridge.cpp_process_bridge import SimulatorRun


class RunViewerFrame(tk.Frame):
    def __init__(self, parent, project_root):
        super().__init__(parent)
        self.project_root = project_root

        self.map_path_var = tk.StringVar(value="")
        self.radius_var = tk.IntVar(value=0)
        self.margin_var = tk.IntVar(value=0)
        self.speed_var = tk.IntVar(value=120)  # ms giua moi buoc khi auto-play

        self.run = None              # SimulatorRun hien tai
        self.current_step = 0
        self.current_state = None    # dict JSON cua step dang hien thi
        self.playing = False
        self.play_after_id = None

        self.show_paths = tk.BooleanVar(value=True)
        self.show_grid_lines = tk.BooleanVar(value=True)

        self._build_ui()

    # ---------------------------------------------------------------- UI ----

    def _build_ui(self):
        def mkbtn(parent, text, cmd, bg="#dddddd", fg="black", state=tk.NORMAL):
            b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg,
                           font=("Arial", 10), padx=6, state=state)
            b.pack(side=tk.LEFT, padx=2)
            return b

        # Row 1: chon map + tham so robot
        r1 = tk.Frame(self, pady=3)
        r1.pack(side=tk.TOP, fill=tk.X, padx=6)

        mkbtn(r1, "Chon file map JSON...", self.choose_map, "#607d8b", "white")
        tk.Label(r1, textvariable=self.map_path_var, font=("Arial", 9), fg="#333333").pack(side=tk.LEFT, padx=6)

        r1b = tk.Frame(self, pady=3)
        r1b.pack(side=tk.TOP, fill=tk.X, padx=6)
        tk.Label(r1b, text="Ban kinh robot (o):", font=("Arial", 10)).pack(side=tk.LEFT)
        tk.Spinbox(r1b, from_=0, to=5, textvariable=self.radius_var, width=3,
                   font=("Arial", 10)).pack(side=tk.LEFT, padx=(2, 10))
        tk.Label(r1b, text="Khoang an toan (o):", font=("Arial", 10)).pack(side=tk.LEFT)
        tk.Spinbox(r1b, from_=0, to=5, textvariable=self.margin_var, width=3,
                   font=("Arial", 10)).pack(side=tk.LEFT, padx=(2, 10))
        tk.Checkbutton(r1b, text="Hien duong di A*", variable=self.show_paths,
                       command=self._redraw, font=("Arial", 10)).pack(side=tk.LEFT, padx=(8, 0))

        # Row 2: dieu khien Start / Pause / Step / Reset
        r2 = tk.Frame(self, pady=3)
        r2.pack(side=tk.TOP, fill=tk.X, padx=6)

        self.start_btn = mkbtn(r2, "Chay mo phong (Start)", self.start_simulation, "#4caf50", "white")
        self.play_btn = mkbtn(r2, "Phat", self.toggle_play, "#2196f3", "white", state=tk.DISABLED)
        self.step_btn = mkbtn(r2, "Step ->", self.step_forward, "#ff9800", "white", state=tk.DISABLED)
        self.step_back_btn = mkbtn(r2, "<- Step", self.step_backward, "#ff9800", "white", state=tk.DISABLED)
        self.reset_btn = mkbtn(r2, "Reset", self.reset_view, "#f44336", "white", state=tk.DISABLED)

        tk.Label(r2, text="  Toc do (ms/buoc):", font=("Arial", 10)).pack(side=tk.LEFT)
        tk.Scale(r2, from_=20, to=1000, orient=tk.HORIZONTAL, variable=self.speed_var,
                 length=120, showvalue=False).pack(side=tk.LEFT)

        # Row 3: thanh trang thai
        r3 = tk.Frame(self, pady=2)
        r3.pack(side=tk.TOP, fill=tk.X, padx=6)
        self.status_var = tk.StringVar(value="Chua chon map. Hay chon file map JSON da xuat tu tab 'Xay dung map'.")
        tk.Label(r3, textvariable=self.status_var, font=("Arial", 10), fg="#333333", anchor="w").pack(fill=tk.X)

        # Row 4: thanh truot timestep
        r4 = tk.Frame(self, pady=2)
        r4.pack(side=tk.TOP, fill=tk.X, padx=6)
        tk.Label(r4, text="Timestep:", font=("Arial", 9)).pack(side=tk.LEFT)
        # KHONG dung command= o day: Tk Scale co the tu goi lai command mot cach BAT DONG BO
        # (o vong event loop ke tiep) ngay ca khi gia tri duoc code tu thay doi qua config(to=)/set(),
        # khien co _updating_scale_internally bi reset truoc khi callback that su chay - gay doc
        # nham mot step cu. Thay vao do, chi phan ung khi nguoi dung THUC SU tha chuot sau khi keo.
        self.step_scale = tk.Scale(r4, from_=0, to=0, orient=tk.HORIZONTAL,
                                   length=400, showvalue=True)
        self.step_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self.step_scale.bind("<ButtonRelease-1>", self._on_scale_release)

        # Legend
        r5 = tk.Frame(self)
        r5.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(0, 2))
        for key, label in [
            ("UNKNOWN", "Chua kham pha"), ("FREE", "Da biet (chua qua)"),
            ("VISITED", "Da di qua"), ("OCCUPIED", "Vat can"),
            ("INFLATED_OBSTACLE", "Vung cam (inflate)"), ("TARGET", "Dich that"),
        ]:
            tk.Label(r5, bg=CELL_STATE_COLOR[key], width=2).pack(side=tk.LEFT, padx=(4, 1))
            tk.Label(r5, text=label, font=("Arial", 9)).pack(side=tk.LEFT, padx=(0, 6))

        # Canvas chinh
        self.canvas = tk.Canvas(self, bg="#1e1e1e", width=680, height=520)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        self.canvas.bind("<Configure>", lambda _: self._redraw())

        # Panel trang thai robot
        self.robot_info_var = tk.StringVar(value="")
        info_label = tk.Label(self, textvariable=self.robot_info_var, font=("Consolas", 9),
                              fg="#222222", anchor="w", justify=tk.LEFT)
        info_label.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(0, 6))

    # ------------------------------------------------------------ Actions ----

    def choose_map(self):
        default_dir = os.path.join(self.project_root, "config", "maps")
        path = filedialog.askopenfilename(
            title="Chon file map JSON", initialdir=default_dir,
            filetypes=[("JSON files", "*.json")],
        )
        if path:
            self.set_map_path(path)

    def set_map_path(self, path):
        self.map_path_var.set(path)
        self.status_var.set(f"Da chon map: {path}. Nhan 'Chay mo phong' de bat dau.")

    def start_simulation(self):
        map_path = self.map_path_var.get()
        if not map_path or not os.path.isfile(map_path):
            messagebox.showwarning("Chua co map", "Hay chon file map JSON hop le truoc.")
            return

        if self.playing:
            self._stop_playback()

        output_dir = os.path.join(self.project_root, "python", "run_output")
        self.run = SimulatorRun(
            self.project_root, map_path, output_dir,
            robot_radius=self.radius_var.get(),
            safety_margin=self.margin_var.get(),
            max_timesteps=200000,
        )
        ok = self.run.start()
        if not ok:
            messagebox.showerror("Loi khoi chay simulator", self.run.error_message or "Loi khong xac dinh.")
            return

        self.current_step = 0
        self.current_state = None
        self.status_var.set("Dang chay simulator C++ (nen)... cho du lieu buoc dau tien.")
        self.play_btn.config(state=tk.NORMAL)
        self.step_btn.config(state=tk.NORMAL)
        self.step_back_btn.config(state=tk.NORMAL)
        self.reset_btn.config(state=tk.NORMAL)

        self._poll_for_first_step()

    def _poll_for_first_step(self):
        steps = self.run.list_available_steps()
        if steps:
            self._load_step(steps[0])
            self._set_scale_range(max(steps))
            self._auto_extend_scale()
            return
        if self.run.finished and self.run.error_message:
            messagebox.showerror("Loi simulator", self.run.error_message)
            self.status_var.set("Simulator gap loi. Xem chi tiet trong hop thoai.")
            return
        self.after(50, self._poll_for_first_step)

    def _auto_extend_scale(self):
        """Khi simulator van dang chay nen, dinh ky mo rong thanh truot theo so step moi co."""
        if self.run is None:
            return
        steps = self.run.list_available_steps()
        if steps:
            self._set_scale_range(max(steps))
        if not self.run.finished:
            self.after(150, self._auto_extend_scale)
        else:
            final = "da hoan tat" if not self.run.error_message else "gap loi"
            self.status_var.set(f"Simulator {final}. Tong so timestep: "
                                f"{(max(steps)+1) if steps else 0}. Dung thanh truot/Step de xem lai.")

    def toggle_play(self):
        if self.playing:
            self._stop_playback()
        else:
            self._start_playback()

    def _start_playback(self):
        if self.run is None:
            return
        self.playing = True
        self.play_btn.config(text="Dung")
        self._play_tick()

    def _stop_playback(self):
        self.playing = False
        self.play_btn.config(text="Phat")
        if self.play_after_id is not None:
            self.after_cancel(self.play_after_id)
            self.play_after_id = None

    def _play_tick(self):
        if not self.playing:
            return
        available = self.run.list_available_steps()
        next_step = self.current_step + 1
        if available and next_step <= max(available):
            self._load_step(next_step)
        elif self.run.finished:
            self._stop_playback()
            self.status_var.set("Da phat het du lieu mo phong.")
            return
        self.play_after_id = self.after(self.speed_var.get(), self._play_tick)

    def step_forward(self):
        if self.run is None:
            return
        available = self.run.list_available_steps()
        nxt = self.current_step + 1
        if available and nxt <= max(available):
            self._load_step(nxt)

    def step_backward(self):
        if self.run is None or self.current_step <= 0:
            return
        self._load_step(self.current_step - 1)

    def reset_view(self):
        if self.run is not None:
            self.run.stop()
        self._stop_playback()
        self.run = None
        self.current_step = 0
        self.current_state = None
        self._set_scale_range(0)
        self._set_scale_value(0)
        self.play_btn.config(state=tk.DISABLED, text="Phat")
        self.step_btn.config(state=tk.DISABLED)
        self.step_back_btn.config(state=tk.DISABLED)
        self.reset_btn.config(state=tk.DISABLED)
        self.canvas.delete("all")
        self.robot_info_var.set("")
        self.status_var.set("Da reset. Chon map va nhan 'Chay mo phong' de bat dau lai.")

    def _on_scale_release(self, event):
        if self.run is None:
            return
        step = int(self.step_scale.get())
        if step != self.current_step:
            self._load_step(step, update_scale=False)

    def _set_scale_range(self, to_value):
        self.step_scale.config(to=to_value)

    def _set_scale_value(self, value):
        self.step_scale.set(value)

    def _load_step(self, step_index, update_scale=True):
        state = self.run.read_step(step_index)
        if state is None:
            return
        self.current_step = step_index
        self.current_state = state
        if update_scale:
            self._set_scale_value(step_index)
        self._redraw()
        self._update_info_panel()

    # -------------------------------------------------------------- Draw ----

    def _cell_size(self, w, h):
        cw = self.canvas.winfo_width() or 680
        ch = self.canvas.winfo_height() or 520
        return max(3, min(cw // max(1, w), ch // max(1, h)))

    def _redraw(self):
        self.canvas.delete("all")
        if not self.current_state:
            return

        w = self.current_state["width"]
        h = self.current_state["height"]
        cells = self.current_state["cells"]
        cs = self._cell_size(w, h)
        outline = C_GRID_LINE if self.show_grid_lines.get() else ""

        for x in range(w):
            for y in range(h):
                state = cells[x][y]
                color = CELL_STATE_COLOR.get(state, "#ff00ff")
                x0 = x * cs
                y0 = (h - 1 - y) * cs
                x1, y1 = x0 + cs, y0 + cs
                self.canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline=outline)

        # Vẽ tường nội bộ đã được khám phá (fog of war: chỉ hiện tường ô đã biết)
        walls = self.current_state.get("walls", {})
        if walls:
            self._draw_walls(walls, w, h, cs)

        # Duong di A* cua tung robot (tuy chon)
        if self.show_paths.get():
            for r in self.current_state["robots"]:
                path = r.get("path", [])
                if len(path) < 2:
                    continue
                color = robot_color(r["id"])
                pts = []
                for (px, py) in path:
                    cx = px * cs + cs // 2
                    cy = (h - 1 - py) * cs + cs // 2
                    pts.extend([cx, cy])
                if len(pts) >= 4:
                    self.canvas.create_line(*pts, fill=color, width=max(1, cs // 8), dash=(4, 2))

        # Ve robot
        for r in self.current_state["robots"]:
            rx, ry = r["x"], r["y"]
            color = robot_color(r["id"])
            x0 = rx * cs
            y0 = (h - 1 - ry) * cs
            x1, y1 = x0 + cs, y0 + cs
            pad = max(1, cs // 6)
            self.canvas.create_oval(x0 + pad, y0 + pad, x1 - pad, y1 - pad,
                                    fill=color, outline="#000000", width=1)
            self.canvas.create_text((x0 + x1) // 2, (y0 + y1) // 2, text=str(r["id"]),
                                    fill="white", font=("Arial", max(7, cs // 2), "bold"))

        self.canvas.create_rectangle(1, 1, w * cs, h * cs, outline="#000000", width=2)

    def _draw_walls(self, walls, w, h, cs):
        """Vẽ tường nội bộ đã được khám phá lên canvas.

        Quy ước hướng (khớp C++ kDirX/kDirY):
          N (dir=0): y+1 → cạnh TRÊN ô (y0 nhỏ hơn vì canvas bị đảo)
          E (dir=1): x+1 → cạnh PHẢI ô
          S (dir=2): y-1 → cạnh DƯỚI ô
          W (dir=3): x-1 → cạnh TRÁI ô
        """
        # (dir_name, lambda tính toạ độ 2 điểm đầu-cuối của đường tường)
        dir_coords = [
            ("N", lambda x0, y0, x1, y1: (x0, y0, x1, y0)),  # cạnh trên
            ("E", lambda x0, y0, x1, y1: (x1, y0, x1, y1)),  # cạnh phải
            ("S", lambda x0, y0, x1, y1: (x0, y1, x1, y1)),  # cạnh dưới
            ("W", lambda x0, y0, x1, y1: (x0, y0, x0, y1)),  # cạnh trái
        ]
        for dir_name, coords_fn in dir_coords:
            wall_grid = walls.get(dir_name)
            if not wall_grid:
                continue
            for x in range(w):
                if x >= len(wall_grid):
                    continue
                col = wall_grid[x]
                for y in range(h):
                    if y >= len(col):
                        continue
                    if col[y]:
                        x0 = x * cs
                        y0 = (h - 1 - y) * cs
                        x1, y1 = x0 + cs, y0 + cs
                        lx0, ly0, lx1, ly1 = coords_fn(x0, y0, x1, y1)
                        self.canvas.create_line(lx0, ly0, lx1, ly1,
                                                fill="#111111", width=3)


    def _update_info_panel(self):
        if not self.current_state:
            return
        status = self.current_state.get("status", "RUNNING")
        ts = self.current_state.get("timestep", 0)
        lines = [f"Timestep: {ts}   |   Trang thai: {STATUS_LABEL_VI.get(status, status)}"]
        for r in self.current_state["robots"]:
            phase = PHASE_LABEL_VI.get(r["phase"], r["phase"])
            goal = f"({r['goal_x']},{r['goal_y']})" if r.get("has_goal") else "-"
            lines.append(f"  Robot {r['id']}: vi tri=({r['x']},{r['y']})  "
                        f"trang thai={phase:18s}  muc tieu={goal:10s}  so buoc da di={r['total_steps']}")
        self.robot_info_var.set("\n".join(lines))
        self.status_var.set(f"Dang xem timestep {ts} - {STATUS_LABEL_VI.get(status, status)}")
