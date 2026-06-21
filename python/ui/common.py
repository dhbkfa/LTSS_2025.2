"""
common.py - Hang so mau sac va tien ich dung chung cho toan bo Python UI.

Theo kien truc trong bao cao (muc 9): Python CHI lam UI - ve ban do, ve robot,
ve o da biet/chua biet, ve duong di, nut Start/Pause/Step/Reset, doc du lieu
tu C++ qua JSON. Toan bo logic mo phong nam ben C++.
"""

# ---- Mau sac dung cho che do XAY DUNG MAP (map builder) ----
C_INACTIVE = "#999999"   # vung khong ton tai trong hinh dang map
C_CELL = "#ffffff"       # o trong (active, khong vat can)
C_OCCUPIED = "#222222"   # o vat can (tuong / chuong ngai)
C_START = "#88ff88"      # vi tri xuat phat cua robot
C_GOAL_BG = "#ffaaaa"    # vi tri dich that
C_WALL = "#111111"
C_GRID_LINE = "#cccccc"

# ---- Mau sac dung cho che do CHAY MO PHONG (run viewer) ----
# Khop voi CellState enum ben C++ (UiProtocol::cellStateStr)
C_UNKNOWN = "#3a3a3a"           # chua kham pha - mau toi, dung y "man suong chien tranh"
C_FREE = "#ffffff"              # da biet, di duoc, chua co robot di qua
C_RUN_OCCUPIED = "#222222"      # vat can da phat hien
C_INFLATED = "#caa6ff"          # vung cam (vat can + ban kinh robot + an toan)
C_VISITED = "#bfe3ff"           # da co robot di qua
C_TARGET = "#ff5050"            # dich that da xac nhan
C_INACTIVE_RUN = "#999999"      # ngoai hinh dang map

CELL_STATE_COLOR = {
    "UNKNOWN": C_UNKNOWN,
    "FREE": C_FREE,
    "OCCUPIED": C_RUN_OCCUPIED,
    "INFLATED_OBSTACLE": C_INFLATED,
    "VISITED": C_VISITED,
    "TARGET": C_TARGET,
    "INACTIVE": C_INACTIVE_RUN,
}

# Mau rieng cho tung robot (toi da 5 theo de tai), du nguoi dung doi so luong vao khac 5
# thi van co the mo rong vong lap mau.
ROBOT_COLORS = [
    "#0055ff",  # robot 0 - xanh duong
    "#ff8800",  # robot 1 - cam
    "#00aa55",  # robot 2 - xanh la
    "#aa00cc",  # robot 3 - tim
    "#cc0000",  # robot 4 - do
]

PHASE_LABEL_VI = {
    "EXPLORING": "Dang kham pha",
    "GOING_TO_TARGET": "Dang den dich",
    "ARRIVED": "Da den dich",
    "WAITING": "Dang cho",
    "UNREACHABLE": "Khong the toi",
}

STATUS_LABEL_VI = {
    "RUNNING": "Dang chay",
    "TARGET_FOUND": "Da tim thay dich",
    "MAP_COVERED": "Da phu het ban do",
    "STUCK": "Bi ket (khong con frontier kha thi)",
}


def robot_color(robot_id: int) -> str:
    return ROBOT_COLORS[robot_id % len(ROBOT_COLORS)]
