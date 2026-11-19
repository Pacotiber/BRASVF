"""Robot arm control application (ESP32 + PCA9685, firmware 06_ESP32/arm_bridge).

  - Control     : go to a position (X, Y, Z) or set each joint with a slider
  - Map         : map of every reachable position; the robot follows the mouse
  - Calibration : limits of each servo and pulse width -> real angle mapping
  - Geometry    : arm dimensions (H0 to be measured on the real robot)

Run:  python arm_app.py      (numpy and Pillow; pyserial for the real robot)
Without a robot, choose "Simulation": everything works with a simulated robot.
"""
import math
import os
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
from PIL import Image, ImageTk

from calibration import DISPLAY_NAMES, US_PER_DEG, Config
from kinematics import (DEG, Geometry, ReachMap, arm_points, fk, follow_path, into_ranges, path_is_safe,
                        posture_is_valid, solve)
from robot_link import PYSERIAL, SIMULATION, SerialLink, SimulatedLink, list_ports

FOLDER = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(FOLDER, "arm_config.json")
OLD_CONFIG_FILE = os.path.join(FOLDER, "config_bras.json")     # older French version, converted once
GRIPPER_MODES = ("free (auto)", "horizontal", "pointing down", "fixed angle")

BACKGROUND = "#f3f4f6"
MAP_BACKGROUND = "#2b2e33"
EMPTY = np.array([58, 62, 70])           # map: unreachable position
RED, YELLOW, GREEN = np.array([214, 69, 56]), np.array([236, 190, 64]), np.array([72, 176, 98])

GEOMETRY_FIELDS = (
    ("L1", "Arm 1: shoulder axis → elbow axis"),
    ("L2", "Arm 2: elbow axis → wrist axis"),
    ("L3", "Gripper: wrist axis → fingertips (along the gripper)"),
    ("HP", "Fingertips offset perpendicular to the gripper"),
    ("R0", "Shoulder axis in front of the base axis"),
    ("H0", "Height of the shoulder axis above the table   ← MEASURE IT on the robot"),
    ("lat_shoulder", "Sideways offset (to the right) at the shoulder"),
    ("lat_elbow", "Sideways offset at the elbow"),
    ("lat_wrist", "Sideways offset at the wrist"),
    ("table_margin", "Safety: no point of the arm below this height"),
    ("servo_radius", "Safety: radius of the elbow and wrist servos around their axis"),
    ("base_radius", "Safety: forbidden zone around the base, radius"),
    ("base_height", "Safety: forbidden zone around the base, height"),
    ("gripper_len_min", "Gripper outline: start, along its axis"),
    ("gripper_perp_min", "Gripper outline: lower side (perpendicular)"),
    ("gripper_perp_max", "Gripper outline: upper side (perpendicular)"),
    ("max_opening", "Finger gap with the gripper open (drawing only)"),
)

# Calibration without typing angles: move the real arm to an easy-to-recognise position, then click the button that
# describes it. Two clearly different positions per joint are enough (offset + direction).
REFERENCES = (
    ((0, "0°: arm pointing forward"), (90, "90°: arm pointing right"), (-90, "−90°: arm pointing left")),
    ((0, "0°: arm 1 horizontal, forward"), (90, "90°: arm 1 vertical"),
     (180, "180°: arm 1 horizontal, backward")),
    ((0, "0°: arm 2 aligned with arm 1"), (-90, "−90°: arm 2 folded DOWN"),
     (90, "+90°: arm 2 folded UP")),
    ((0, "0°: gripper aligned with arm 2"), (-90, "−90°: gripper folded DOWN"),
     (90, "+90°: gripper folded UP")),
)

JOINT_HELP = (
    "Base: with +10 / −10, turn the arm forward, then click the « 0° » button. Then turn it to the right (or to "
    "the left) and click the matching button. Finally record both limits.",
    "Shoulder: put arm 1 horizontal, pointing forward, click « 0° ». Put it vertical, click « 90° ». Finally record "
    "both limits.",
    "Elbow: first put the shoulder horizontal. Align arm 2 with arm 1, click « 0° ». Fold it at a right angle and "
    "click « DOWN » (towards the table) or « UP ». Then both limits.",
    "Wrist: align the gripper with arm 2, click « 0° ». Fold it at a right angle and click « DOWN » (towards the "
    "table when arm 2 is horizontal) or « UP ». Then both limits.",
    "Gripper: find the pulse width where the gripper is wide open and click « Open = here ». Then find the one where "
    "the fingers just touch and click « Closed = here ». Do not force: a stalled SG90 heats up.",
)


def map_colours(ok, margin):
    """RGB image of the map. Grey = unreachable. Otherwise the colour shows the margin to the limits:
    red (0°), then yellow (10°), then green (25° and more)."""
    m = np.clip(margin, 0.0, 25.0)[..., None]
    c = np.where(m < 10.0, RED + (m / 10.0) * (YELLOW - RED), YELLOW + ((m - 10.0) / 15.0) * (GREEN - YELLOW))
    return np.where(np.asarray(ok)[..., None], c, EMPTY).astype(np.uint8)


def number(text):
    return float(str(text).replace(",", ".").strip())


# ==================================================================================== 3D view
class View3D(tk.Canvas):
    CENTRE = np.array([130.0, 150.0, 0.0])

    def __init__(self, parent, app):
        super().__init__(parent, bg=BACKGROUND, highlightthickness=0)
        self.app = app
        self.az, self.el, self.zoom = 40.0, 24.0, 1.0
        self._mouse = None
        self.bind("<Configure>", lambda e: self.draw())
        self.bind("<ButtonPress-1>", lambda e: setattr(self, "_mouse", (e.x, e.y)))
        self.bind("<B1-Motion>", self._rotate)
        self.bind("<MouseWheel>", self._wheel)

    def _rotate(self, e):
        if self._mouse:
            self.az = (self.az - (e.x - self._mouse[0]) * 0.4) % 360.0
            self.el = min(89.0, max(-5.0, self.el + (e.y - self._mouse[1]) * 0.4))
            self._mouse = (e.x, e.y)
            self.draw()

    def _wheel(self, e):
        self.zoom = min(4.0, max(0.3, self.zoom * (1.1 if e.delta > 0 else 1 / 1.1)))
        self.draw()

    def _prepare(self):
        a, e = self.az * DEG, self.el * DEG
        d = np.array([math.cos(e) * math.cos(a), math.sin(e), math.cos(e) * math.sin(a)])   # towards the camera
        self.right = np.cross([0.0, 1.0, 0.0], d)
        self.right /= np.linalg.norm(self.right)
        self.up = np.cross(d, self.right)
        w, h = self.winfo_width(), self.winfo_height()
        self.s = self.zoom * min(w, h) / 600.0
        self.cx, self.cy = w / 2.0, h / 2.0 + 40

    def proj(self, P):
        P = np.atleast_2d(np.asarray(P, dtype=float)) - self.CENTRE
        return np.stack([self.cx + self.s * (P @ self.right), self.cy - self.s * (P @ self.up)], axis=1)

    def line(self, pts, colour, width=1, **kw):
        uv = self.proj(pts)
        if len(uv) >= 2:
            self.create_line(*uv.ravel().tolist(), fill=colour, width=width, capstyle="round",
                             joinstyle="round", **kw)

    def disc(self, P, r, **kw):
        u, v = self.proj(P)[0]
        self.create_oval(u - r, v - r, u + r, v + r, **kw)

    def _cylinder(self, radius, height, colour, top_colour):
        t = np.linspace(0, 2 * np.pi, 37)
        bottom = np.stack([radius * np.cos(t), np.zeros_like(t), radius * np.sin(t)], axis=1)
        top = bottom + [0.0, height, 0.0]
        ub, ut = self.proj(bottom), self.proj(top)
        for k in range(36):
            self.create_polygon(*ub[k], *ub[k + 1], *ut[k + 1], *ut[k], fill=colour, outline="")
        self.create_polygon(*ut.ravel().tolist(), fill=top_colour, outline="#6b7280")

    def _arm(self, q, pct, ghost):
        g = self.app.cfg.geometry
        pts, frame, _ = arm_points(g, q)
        cl = "#a9b4c2" if ghost else "#1f4e79"
        cs = "#b8c1cc" if ghost else "#30343b"
        cg = "#c9b79c" if ghost else "#b45309"
        shadow = [pts[k] * [1, 0, 1] for k in ("shoulder", "elbow", "wrist", "tip")]
        if not ghost:
            self.line(shadow, "#d6dae1", 4)
        self.line([[0, 52, 0], pts["shoulder"]], cl, 9 if not ghost else 5)
        self.line([pts["shoulder"], pts["shoulder2"], pts["elbow"]], cl, 7 if not ghost else 4)
        self.line([pts["elbow"], pts["elbow2"], pts["wrist"]], cl, 6 if not ghost else 4)
        self.line([pts["wrist"], pts["wrist2"]], cl, 5 if not ghost else 3)
        for k in ("shoulder2", "elbow2", "wrist2"):
            self.disc(pts[k], max(3.0, 13 * self.s), fill=cs, outline="")
        # gripper: palm and two fingers opening sideways
        opening = g.max_opening * max(0.0, min(100.0, pct)) / 200.0 + 2.0
        palm = pts["wrist2"] + frame["approach"] * g.gripper_len_min + frame["perp"] * g.HP
        fingers = []
        for sgn in (-1, 1):
            o = frame["side"] * sgn * opening
            fingers.append((palm + o, pts["tip"] + o))
        self.line([pts["wrist2"], palm], cg, 5 if not ghost else 3)
        self.line([fingers[0][0], fingers[1][0]], cg, 4 if not ghost else 2)
        for a, b in fingers:
            self.line([a, b], cg, 3 if not ghost else 2)

    def draw(self):
        self.delete("all")
        if self.winfo_width() < 20:
            return
        self._prepare()
        app = self.app
        t = np.linspace(0, 2 * np.pi, 73)
        for r in (100, 200, 300, 400):
            self.line(np.stack([r * np.cos(t), 0 * t, r * np.sin(t)], 1), "#d5d9e0", 1)
        for k in range(12):
            a = k * 30 * DEG
            self.line([[0, 0, 0], [420 * math.cos(a), 0, 420 * math.sin(a)]], "#e1e4ea", 1)
        for (P, colour, name) in (((130, 0, 0), "#d03030", "X (forward)"), ((0, 0, 130), "#2f62d0", "Z (right)"),
                                  ((0, 130, 0), "#2a9a3a", "Y")):
            self.line([[0, 0, 0], P], colour, 2, arrow="last")
            u, v = self.proj(P)[0]
            self.create_text(u + 4, v - 4, text=name, fill=colour, anchor="sw", font=("Segoe UI", 9))
        self._cylinder(50.0, 52.0, "#b9bec6", "#d3d7dd")
        q_real = app.q_real_full()
        if q_real is None:
            self._arm(app.q_cmd, app.gripper_pct, True)
        else:
            if max(abs(a - b) for a, b in zip(q_real, app.q_cmd)) > 0.5:
                self._arm(app.q_cmd, app.gripper_pct, True)
            self._arm(q_real, app.gripper_real if app.gripper_real is not None else app.gripper_pct, False)
        if app.target is not None:
            c = np.asarray(app.target, dtype=float)
            colour = "#16a34a" if app.target_ok else "#dc2626"
            self.line([c, c * [1, 0, 1]], colour, 1, dash=(3, 3))
            self.disc(c * [1, 0, 1], 3, fill=colour, outline="")
            self.disc(c, 6, outline=colour, width=2)
        state = "" if q_real is not None else "  —  servos released: the grey arm is the command (preview)"
        self.create_text(10, 10, anchor="nw", fill="#6b7280", font=("Segoe UI", 9),
                         text="Drag: rotate the view    Wheel: zoom" + state)


# ==================================================================================== maps
class MapView(tk.Canvas):
    STEP = 3     # pixels per computed cell

    def __init__(self, parent, app, kind):
        super().__init__(parent, bg=MAP_BACKGROUND, highlightthickness=0, cursor="crosshair")
        self.app, self.kind = app, kind
        self._photo = None
        self.bind("<Configure>", lambda e: self.draw(True))
        self.bind("<Motion>", lambda e: app.mouse_on_map(self, e.x, e.y))
        self.bind("<Leave>", lambda e: app.mouse_left_map(self))
        self.bind("<Button-1>", lambda e: app.map_click(self))
        self.bind("<Button-3>", lambda e: app.map_go(self, e.x, e.y))
        self.bind("<MouseWheel>", lambda e: app.map_wheel(self, e.delta))
        # space bar = open / close the gripper while the mouse is on the map (the map then takes the keyboard:
        # Space does not trigger the last clicked button)
        self.bind("<Enter>", lambda e: self.focus_set(), add="+")
        self.bind("<Leave>", lambda e: app.focus_set() if self.focus_get() is self else None, add="+")
        self.bind("<space>", lambda e: (app.toggle_gripper(), "break")[1])

    def frame(self):
        w, h = max(self.winfo_width(), 50), max(self.winfo_height(), 50)
        g = self.app.cfg.geometry
        R = g.reach + 15
        if self.kind == "top":
            return (min(w, h) - 30) / (2 * R), w / 2.0, h / 2.0
        s = min((w - 30) / (2 * R), (h - 45) / (g.H0 + R))
        return s, w / 2.0, h - 24.0

    def screen(self, A, B):
        """Top view: A = X (towards the top of the screen), B = Z (to the right).
        Side view: A = reach in the direction of the view, B = height Y."""
        s, cx, cy = self.frame()
        return (cx + s * B, cy - s * A) if self.kind == "top" else (cx + s * A, cy - s * B)

    def point3d(self, u, v):
        s, cx, cy = self.frame()
        if self.kind == "top":
            return (cy - v) / s, self.app.map_height, (u - cx) / s
        a, th = (u - cx) / s, self.app.azimuth
        return a * math.cos(th), max(0.0, (cy - v) / s), a * math.sin(th)

    def section(self, P):
        if self.kind == "top":
            return P[0], P[2]
        th = self.app.azimuth
        return P[0] * math.cos(th) + P[2] * math.sin(th), P[1]

    def draw(self, image=False):
        if image:
            self._image()
        self.delete("over")
        self._overlay()

    def _image(self):
        w, h = self.winfo_width(), self.winfo_height()
        self.delete("img")
        if w < 20 or h < 20:
            return
        reach_map = self.app.current_map()
        if reach_map is None:
            self.create_text(w / 2, h / 2, text="Computing the map of reachable positions...",
                             fill="#e5e7eb", font=("Segoe UI", 12), tags="img")
            return
        k = self.STEP
        U, V = np.meshgrid(np.arange(0, w, k) + k / 2.0, np.arange(0, h, k) + k / 2.0)
        s, cx, cy = self.frame()
        if self.kind == "top":
            X, Y, Z = (cy - V) / s, self.app.map_height, (U - cx) / s
        else:
            a = (U - cx) / s
            X, Y, Z = a * math.cos(self.app.azimuth), (cy - V) / s, a * math.sin(self.app.azimuth)
        ok, margin = reach_map.point(X, Y, Z)
        img = Image.fromarray(map_colours(ok, margin))
        self._photo = ImageTk.PhotoImage(img.resize((img.width * k, img.height * k), Image.Resampling.NEAREST))
        self.create_image(0, 0, image=self._photo, anchor="nw", tags="img")
        self.tag_lower("img")

    def _overlay(self):
        app, T = self.app, "over"
        g = app.cfg.geometry
        s, cx, cy = self.frame()
        R = g.reach + 15
        grey, text = "#7b818b", "#c7ccd4"
        font = ("Segoe UI", 8)
        if self.kind == "top":
            for r in (100, 200, 300, 400):
                self.create_oval(cx - r * s, cy - r * s, cx + r * s, cy + r * s, outline=grey, tags=T)
                self.create_text(cx + 3, cy - r * s, text=str(r), fill=text, anchor="sw", font=font, tags=T)
            for b in app.lim[0]:
                u, v = self.screen(R * math.cos(b * DEG), R * math.sin(b * DEG))
                self.create_line(cx, cy, u, v, fill="#f59e0b", dash=(5, 3), tags=T)
                self.create_text(u, v, text="base limit %+.0f°" % b, fill="#f59e0b", font=font, tags=T,
                                 anchor="s" if v > cy else "n")
            u, v = self.screen(R * math.cos(app.azimuth), R * math.sin(app.azimuth))
            self.create_line(cx, cy, u, v, fill="#e5e7eb", dash=(2, 4), tags=T)
            r = g.base_radius * s
            self.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#e5e7eb", dash=(3, 3), tags=T)
            self.create_line(cx, cy, cx, cy - 55, fill="#ef4444", width=2, arrow="last", tags=T)
            self.create_text(cx + 4, cy - 55, text="X forward", fill="#fca5a5", anchor="w", font=font, tags=T)
            self.create_line(cx, cy, cx + 55, cy, fill="#60a5fa", width=2, arrow="last", tags=T)
            self.create_text(cx + 55, cy + 4, text="Z right", fill="#93c5fd", anchor="n", font=font, tags=T)
        else:
            for y in range(100, int(g.H0 + R), 100):
                u0, v0 = self.screen(-R, y)
                u1, _ = self.screen(R, y)
                self.create_line(u0, v0, u1, v0, fill=grey, dash=(1, 4), tags=T)
                self.create_text(u0 + 2, v0, text=str(y), fill=text, anchor="sw", font=font, tags=T)
            for a in (-300, -200, -100, 100, 200, 300):
                u0, v0 = self.screen(a, 0)
                _, v1 = self.screen(a, g.H0 + R)
                self.create_line(u0, v0, u0, v1, fill=grey, dash=(1, 4), tags=T)
                self.create_text(u0, v0 + 2, text=str(a), fill=text, anchor="n", font=font, tags=T)
            u0, v0 = self.screen(-R, 0)
            u1, _ = self.screen(R, 0)
            self.create_line(u0, v0, u1, v0, fill="#d1d5db", width=2, tags=T)
            u0, v0 = self.screen(-g.base_radius, g.base_height)
            u1, v1 = self.screen(g.base_radius, 0)
            self.create_rectangle(u0, v0, u1, v1, outline="#e5e7eb", dash=(3, 3), tags=T)
            u0, v0 = self.screen(-R, app.map_height)
            u1, _ = self.screen(R, app.map_height)
            self.create_line(u0, v0, u1, v0, fill="#e5e7eb", dash=(2, 4), tags=T)
            self.create_text(u1 - 2, v0 - 2, text="top view section", fill=text, anchor="se", font=font, tags=T)
        # arm (the robot if its position is known, otherwise the command)
        q = app.q_shown()
        pts, _, _ = arm_points(g, q)
        names = ("ground", "shoulder", "shoulder2", "elbow", "elbow2", "wrist", "wrist2", "offset", "tip")
        uv = [self.screen(*self.section(pts[k])) for k in names]
        self.create_line(*[c for p in uv for c in p], fill="white", width=3, capstyle="round",
                         joinstyle="round", tags=T)
        for i in (2, 4, 6):
            self.create_oval(uv[i][0] - 4, uv[i][1] - 4, uv[i][0] + 4, uv[i][1] + 4, fill="#30343b",
                             outline="white", tags=T)
        tu, tv = uv[-1]
        self.create_oval(tu - 5, tv - 5, tu + 5, tv + 5, fill="#22d3ee", outline="white", width=2, tags=T)
        if app.target is not None:
            cu, cv = self.screen(*self.section(app.target))
            colour = "#4ade80" if app.target_ok else "#f87171"
            self.create_line(cu - 9, cv, cu + 9, cv, fill=colour, width=2, tags=T)
            self.create_line(cu, cv - 9, cu, cv + 9, fill=colour, width=2, tags=T)
        if not app.follow_var.get():
            self.create_text(10, 10, anchor="nw", text="FOLLOWING PAUSED (left-click to resume)",
                             fill="#fbbf24", font=("Segoe UI", 10, "bold"), tags=T)


# ==================================================================================== application
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot arm — control")
        self.geometry("1420x880")
        self.minsize(1150, 720)
        try:
            self.state("zoomed")         # full screen (Windows): bigger maps
        except tk.TclError:
            pass
        self.configure(bg=BACKGROUND)
        config_error = ""
        try:
            self.cfg = Config.load(CONFIG_FILE, OLD_CONFIG_FILE)
        except Exception as e:                       # damaged file: start again from the default values
            self.cfg = Config()
            config_error = "arm_config.json unreadable (%s): default values" % e
        self.lim = self.cfg.limits()
        self.q_cmd = into_ranges(self.cfg.rest, self.lim)   # command (model angles)
        self.gripper_pct = 50.0
        self.q_real = [None] * 4                     # robot position reported by the ESP32 (None = released)
        self.gripper_real = None
        self.us_real = None
        self.active = False                          # the 4 arm servos are holding
        self.allowed = False                         # the user has activated the servos
        self.link = None
        self.bridge_ok = False
        self.pca_state, self.i2c_errors = "", 0
        self.move_queue = []                         # steps of a "Go" move (M command)
        self.target, self.target_ok = None, True     # aimed point (display)
        self.follow_target = None                    # last mouse position to follow
        self.follow_info = ""
        self.mouse_view = None
        self.map_height, self.azimuth = 150.0, 0.0
        self.maps, self._computing, self._maps_ready = {}, set(), queue.Queue()
        self._to_send = False
        self._jog = None
        self._auto_update = False
        self._first_pos = False
        self._m_running = False                      # "Go" move (M command) in progress
        self._gripper_after_m = False                # gripper to send at the end of that move
        self._images_to_redo = {"top": True, "side": True}
        self._t = {"send": 0.0, "follow": 0.0, "ping": 0.0, "answer": 0.0, "draw": 0.0, "jog": 0.0,
                   "top": 0.0, "side": 0.0}
        self._to_redraw = True

        self._style()
        self._build()
        self.bind_all("<Escape>", lambda e: self.release())
        self.protocol("WM_DELETE_WINDOW", self.quit_app)
        self._list_ports()
        self.port_var.set(SIMULATION)
        self.connect()
        self._update_sliders()
        self._limits_changed(save=False)
        if config_error:
            self.message(config_error, "error")
        elif self.cfg.problems():
            self.message("Configuration: " + "; ".join(self.cfg.problems()), "error")
        elif not PYSERIAL:
            self.message("Simulation mode. To control the real robot, install pyserial: type  pip install pyserial  "
                         "in a terminal, then restart the application.", "error")
        self.after(20, self._tick)

    # ------------------------------------------------------------------ building the window
    def _style(self):
        st = ttk.Style(self)
        if "vista" in st.theme_names():
            st.theme_use("vista")
        st.configure("Title.TLabel", font=("Segoe UI", 10, "bold"))
        st.configure("Big.TLabel", font=("Segoe UI", 18, "bold"))
        st.configure("Alert.TLabel", foreground="#b91c1c")

    def _build(self):
        bar = tk.Frame(self, bg="#e5e7eb", padx=8, pady=6)
        bar.pack(fill="x")
        # right-hand buttons first: they stay visible even if the status text is long
        tk.Button(bar, text="  RELEASE (Esc)  ", bg="#c62828", fg="white", activebackground="#8e0000",
                  activeforeground="white", font=("Segoe UI", 10, "bold"), relief="flat",
                  command=self.release).pack(side="right")
        ttk.Button(bar, text="Activate servos", command=self.activate).pack(side="right", padx=8)
        tk.Label(bar, text="Port:", bg="#e5e7eb").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var, width=30, state="readonly")
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(bar, text="↻", width=3, command=self._list_ports).pack(side="left")
        ttk.Button(bar, text="Connect", command=self.connect).pack(side="left", padx=4)
        self.status_lab = tk.Label(bar, text="", bg="#e5e7eb", font=("Segoe UI", 10, "bold"), anchor="w")
        self.status_lab.pack(side="left", padx=10, fill="x", expand=True)

        self.msg_lab = tk.Label(self, text="", anchor="w", bg="#e5e7eb", padx=10, pady=4, font=("Segoe UI", 10))
        self.msg_lab.pack(side="bottom", fill="x")

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=6, pady=6)
        self.tabs = nb
        self._control_tab(nb)
        self._map_tab(nb)
        self._calibration_tab(nb)
        self._geometry_tab(nb)
        nb.bind("<<NotebookTabChanged>>", lambda e: self._tab_changed())

    def _control_tab(self, nb):
        f = ttk.Frame(nb)
        nb.add(f, text="  Control  ")
        p = ttk.Frame(f, padding=10)
        p.pack(side="right", fill="y")
        self.view3d = View3D(f, self)
        self.view3d.pack(side="left", fill="both", expand=True)

        lf = ttk.LabelFrame(p, text="Go to a position (mm)", padding=8)
        lf.pack(fill="x")
        self.coord = [tk.StringVar(value=v) for v in ("220", "150", "0")]
        for k, (name, hint) in enumerate((("X", "forward"), ("Y", "height"), ("Z", "right"))):
            ttk.Label(lf, text=name).grid(row=0, column=2 * k, sticky="e")
            e = ttk.Entry(lf, textvariable=self.coord[k], width=7)
            e.grid(row=0, column=2 * k + 1, padx=(3, 10))
            e.bind("<Return>", lambda ev: self.go_to_coordinates())
            ttk.Label(lf, text=hint, foreground="#6b7280").grid(row=1, column=2 * k + 1, sticky="n")
        ttk.Label(lf, text="Gripper").grid(row=2, column=0, sticky="e", pady=(8, 0))
        self.gripper_mode = tk.StringVar(value=GRIPPER_MODES[0])
        cb = ttk.Combobox(lf, textvariable=self.gripper_mode, values=GRIPPER_MODES, state="readonly", width=14)
        cb.grid(row=2, column=1, columnspan=3, sticky="w", padx=(3, 0), pady=(8, 0))
        cb.bind("<<ComboboxSelected>>", lambda e: self._gripper_mode_changed())
        self.gripper_angle = tk.StringVar(value="-45")
        e = ttk.Entry(lf, textvariable=self.gripper_angle, width=6)
        e.grid(row=2, column=4, sticky="e", pady=(8, 0))
        e.bind("<Return>", lambda ev: self._gripper_mode_changed())
        ttk.Label(lf, text="°").grid(row=2, column=5, sticky="w", pady=(8, 0))
        ttk.Button(lf, text="Go", command=self.go_to_coordinates).grid(row=3, column=0, columnspan=3,
                                                                       sticky="ew", pady=(10, 0))
        ttk.Button(lf, text="← current position", command=self._current_coords).grid(
            row=3, column=3, columnspan=3, sticky="ew", padx=(6, 0), pady=(10, 0))

        lf = ttk.LabelFrame(p, text="Joints (degrees)", padding=8)
        lf.pack(fill="x", pady=(12, 0))
        self.sliders, self.slider_values = [], []
        for i, name in enumerate(DISPLAY_NAMES[:4]):
            ttk.Label(lf, text=name, width=8).grid(row=i, column=0, sticky="w")
            s = ttk.Scale(lf, from_=self.lim[i][0], to=self.lim[i][1], length=220,
                          command=lambda v, i=i: self._slider(i, v))
            s.grid(row=i, column=1, pady=4)
            lab = ttk.Label(lf, width=8, anchor="e")
            lab.grid(row=i, column=2)
            self.sliders.append(s)
            self.slider_values.append(lab)
        ttk.Label(lf, text="Gripper", width=8).grid(row=4, column=0, sticky="w")
        self.gripper_slider = ttk.Scale(lf, from_=0, to=100, length=220, command=self._gripper_slider)
        self.gripper_slider.grid(row=4, column=1, pady=4)
        self.gripper_value = ttk.Label(lf, width=8, anchor="e")
        self.gripper_value.grid(row=4, column=2)
        ttk.Label(lf, text="A move that would hit the table or the base is refused.", foreground="#6b7280").grid(
            row=5, column=0, columnspan=3, sticky="w", pady=(4, 0))

        lf = ttk.LabelFrame(p, text="Motion", padding=8)
        lf.pack(fill="x", pady=(12, 0))
        ttk.Label(lf, text="Speed").grid(row=0, column=0, sticky="w")
        self.speed_slider = ttk.Scale(lf, from_=10, to=180, length=180, command=self._speed)
        self.speed_slider.grid(row=0, column=1, padx=4)
        self.speed_value = ttk.Label(lf, width=9, anchor="e")
        self.speed_value.grid(row=0, column=2)
        ttk.Button(lf, text="Rest position", command=self.go_rest).grid(row=1, column=0, columnspan=2,
                                                                        sticky="ew", pady=(8, 0))
        ttk.Button(lf, text="Save as rest position", command=self.store_rest).grid(
            row=1, column=2, sticky="ew", padx=(6, 0), pady=(8, 0))

        self.control_info = tk.Label(p, justify="left", anchor="nw", font=("Consolas", 9), bg="white",
                                     relief="solid", bd=1, padx=8, pady=6)
        self.control_info.pack(fill="both", expand=True, pady=(12, 0))

    def _map_tab(self, nb):
        f = ttk.Frame(nb, padding=6)
        nb.add(f, text="  Map  ")
        top = ttk.Frame(f)
        top.pack(fill="x")
        self.follow_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="The robot follows the mouse", variable=self.follow_var,
                        command=lambda: self._redraw_maps()).pack(side="left")
        ttk.Label(top, text="      Gripper").pack(side="left")
        cb = ttk.Combobox(top, textvariable=self.gripper_mode, values=GRIPPER_MODES, state="readonly", width=14)
        cb.pack(side="left", padx=4)
        cb.bind("<<ComboboxSelected>>", lambda e: self._gripper_mode_changed())
        ttk.Entry(top, textvariable=self.gripper_angle, width=6).pack(side="left")
        ttk.Label(top, text="°").pack(side="left")
        self.gripper_button = ttk.Button(top, text="Open / close the gripper (Space)", command=self.toggle_gripper)
        self.gripper_button.pack(side="left", padx=(16, 0))
        legend = ttk.Frame(top)
        legend.pack(side="right", padx=6)
        tk.Frame(legend, bg="#%02x%02x%02x" % tuple(EMPTY), width=22, height=14).pack(side="left")
        ttk.Label(legend, text=" unreachable      margin to the limits:  0°").pack(side="left")
        gradient = tk.Canvas(legend, width=150, height=14, highlightthickness=0)
        gradient.pack(side="left", padx=4)
        for k in range(150):
            c = map_colours(np.array([True]), np.array([k * 25.0 / 140]))[0]
            gradient.create_line(k, 0, k, 14, fill="#%02x%02x%02x" % tuple(c))
        ttk.Label(legend, text="25° and more").pack(side="left")

        body = ttk.Frame(f)
        body.pack(fill="both", expand=True, pady=6)
        body.columnconfigure(0, weight=1, uniform="c")
        body.columnconfigure(1, weight=1, uniform="c")
        body.rowconfigure(1, weight=1)
        self.top_title = ttk.Label(body, style="Title.TLabel")
        self.top_title.grid(row=0, column=0, sticky="w")
        self.side_title = ttk.Label(body, style="Title.TLabel")
        self.side_title.grid(row=0, column=1, sticky="w")
        self.top_map = MapView(body, self, "top")
        self.top_map.grid(row=1, column=0, sticky="nsew", padx=(0, 4))
        self.side_map = MapView(body, self, "side")
        self.side_map.grid(row=1, column=1, sticky="nsew", padx=(4, 0))
        self.map_info = ttk.Label(f, font=("Consolas", 10))
        self.map_info.pack(fill="x")
        self.follow_info_lab = ttk.Label(f, foreground="#b45309", font=("Segoe UI", 10, "bold"))
        self.follow_info_lab.pack(fill="x")
        ttk.Label(f, foreground="#4b5563", text="Hover: the robot follows the mouse   •   Space: open / close the "
                  "gripper   •   left-click: pause / resume   •   right-click: go here   •   wheel: height (top "
                  "view) or direction (side view)").pack(fill="x")

    def _calibration_tab(self, nb):
        f = ttk.Frame(nb, padding=10)
        nb.add(f, text="  Calibration  ")
        left = ttk.Frame(f)
        left.pack(side="left", fill="y")
        right = ttk.Frame(f)
        right.pack(side="left", fill="both", expand=True, padx=(14, 0))

        lf = ttk.LabelFrame(left, text="Servo to set up", padding=8)
        lf.pack(fill="x")
        self.cal_j = tk.IntVar(value=0)
        for j, name in enumerate(DISPLAY_NAMES):
            ttk.Radiobutton(lf, text=name, value=j, variable=self.cal_j, command=self._cal_select).grid(
                row=0, column=j, padx=4, sticky="w")
        ttk.Label(lf, text="PCA9685 channel:").grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        self.cal_channel = tk.StringVar()
        ttk.Spinbox(lf, from_=0, to=4, width=4, textvariable=self.cal_channel, state="readonly",
                    command=self._cal_channel_changed).grid(row=1, column=2, sticky="w", pady=(8, 0))

        lf = ttk.LabelFrame(left, text="Pulse width sent to the servo", padding=8)
        lf.pack(fill="x", pady=(10, 0))
        self.cal_us = 1500.0
        self.cal_us_lab = ttk.Label(lf, style="Big.TLabel")
        self.cal_us_lab.grid(row=0, column=0, columnspan=6)
        self.cal_angle_lab = ttk.Label(lf, foreground="#4b5563")
        self.cal_angle_lab.grid(row=1, column=0, columnspan=6)
        self.cal_slider = ttk.Scale(lf, from_=500, to=2500, length=400, command=self._cal_slider)
        self.cal_slider.grid(row=2, column=0, columnspan=6, pady=(6, 0))
        for k, d in enumerate((-50, -10, -1, 1, 10, 50)):
            ttk.Button(lf, text="%+d" % d, width=6, command=lambda d=d: self._cal_set(self.cal_us + d)).grid(
                row=3, column=k, padx=1, pady=(6, 0))
        self.cal_send = tk.BooleanVar(value=True)
        ttk.Checkbutton(lf, text="Send to the servo live (limited speed)", variable=self.cal_send).grid(
            row=4, column=0, columnspan=6, sticky="w", pady=(6, 0))

        self.cal_joint_frame = ttk.Frame(left)
        self.cal_right_frame = ttk.Frame(right)        # check and limits, on the right (space)
        lf = ttk.LabelFrame(self.cal_right_frame, text="Limits", padding=8)
        lf.pack(side="bottom", fill="x", pady=(10, 0))
        ttk.Button(lf, text="Low limit = here", command=lambda: self._cal_limit(0)).grid(row=0, column=0, sticky="ew")
        ttk.Button(lf, text="High limit = here", command=lambda: self._cal_limit(1)).grid(row=0, column=1, sticky="ew",
                                                                                          padx=6)
        ttk.Button(lf, text="Go to low", command=lambda: self._cal_go_limit(0)).grid(row=0, column=2, sticky="ew")
        ttk.Button(lf, text="Go to high", command=lambda: self._cal_go_limit(1)).grid(row=0, column=3, sticky="ew",
                                                                                      padx=(6, 0))
        self.cal_limits_lab = ttk.Label(lf)
        self.cal_limits_lab.grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))

        # position buttons (on the right): the real arm is in this position -> calibration point
        lf = ttk.LabelFrame(self.cal_right_frame, text="Which position is the real arm in? (adds a point)",
                            padding=8)
        lf.pack(side="top", fill="x")
        self.cal_ref_buttons = []
        for k in range(3):
            b = ttk.Button(lf, command=lambda k=k: self._cal_add_ref(k))
            b.grid(row=0, column=k, sticky="ew", padx=(0 if k == 0 else 6, 0))
            lf.columnconfigure(k, weight=1)
            self.cal_ref_buttons.append(b)
        other = ttk.Frame(lf)
        other.grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Label(other, text="Other measured angle:").pack(side="left")
        self.cal_measure = tk.StringVar()
        e = ttk.Entry(other, textvariable=self.cal_measure, width=8)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda ev: self._cal_add())
        ttk.Label(other, text="°").pack(side="left")
        ttk.Button(other, text="Add", command=self._cal_add).pack(side="left", padx=(8, 0))
        self.cal_help_lab = ttk.Label(lf, wraplength=820, justify="left", foreground="#374151")
        self.cal_help_lab.grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))

        lf = ttk.LabelFrame(self.cal_joint_frame, text="Calibration points", padding=8)
        lf.pack(fill="x", pady=(10, 0))
        self.cal_list = ttk.Treeview(lf, columns=("us", "angle", "dev"), show="headings", height=5,
                                     selectmode="browse")
        for c, t, w in (("us", "Pulse (µs)", 110), ("angle", "Real angle (°)", 110),
                        ("dev", "Deviation from a line (°)", 160)):
            self.cal_list.heading(c, text=t)
            self.cal_list.column(c, width=w, anchor="center")
        self.cal_list.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(6, 0))
        ttk.Button(lf, text="Delete point", command=self._cal_delete).grid(row=3, column=0, columnspan=2,
                                                                           sticky="w", pady=(6, 0))
        ttk.Button(lf, text="Go to this point", command=self._cal_go_point).grid(row=3, column=2, columnspan=2,
                                                                                sticky="e", pady=(6, 0))
        self.cal_summary = ttk.Label(lf, wraplength=420, justify="left")
        self.cal_summary.grid(row=4, column=0, columnspan=4, sticky="w", pady=(6, 0))

        lf = ttk.LabelFrame(self.cal_right_frame, text="Check / rough setting", padding=8)
        lf.pack(side="top", fill="x")
        ttk.Label(lf, text="Go to angle:").grid(row=0, column=0, sticky="w")
        self.cal_test = tk.StringVar(value="0")
        e = ttk.Entry(lf, textvariable=self.cal_test, width=8)
        e.grid(row=0, column=1, padx=4)
        e.bind("<Return>", lambda ev: self._cal_go_angle())
        ttk.Label(lf, text="°").grid(row=0, column=2, sticky="w")
        ttk.Button(lf, text="Go", command=self._cal_go_angle).grid(row=0, column=3, padx=(8, 0))
        ttk.Label(lf, text="Without points: 1500 µs =").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.cal_zero = tk.StringVar()
        ttk.Entry(lf, textvariable=self.cal_zero, width=8).grid(row=1, column=1, padx=4, pady=(8, 0))
        ttk.Label(lf, text="°   direction").grid(row=1, column=2, sticky="w", pady=(8, 0))
        self.cal_direction = tk.StringVar()
        ttk.Combobox(lf, textvariable=self.cal_direction, values=("+1", "-1"), width=4, state="readonly").grid(
            row=1, column=3, sticky="w", padx=(8, 0), pady=(8, 0))
        ttk.Button(lf, text="Apply", command=self._cal_rough).grid(row=1, column=4, padx=(8, 0), pady=(8, 0))

        self.cal_gripper_frame = ttk.Frame(left)
        lf = ttk.LabelFrame(self.cal_gripper_frame, text="Gripper", padding=8)
        lf.pack(fill="x", pady=(10, 0))
        ttk.Button(lf, text="Open = here", command=lambda: self._cal_gripper(0)).grid(row=0, column=0, sticky="ew")
        ttk.Button(lf, text="Closed = here", command=lambda: self._cal_gripper(1)).grid(row=0, column=1, sticky="ew",
                                                                                       padx=6)
        ttk.Button(lf, text="Open", command=lambda: self._cal_set(self.cfg.gripper.us_open)).grid(
            row=0, column=2, sticky="ew")
        ttk.Button(lf, text="Close", command=lambda: self._cal_set(self.cfg.gripper.us_closed)).grid(
            row=0, column=3, sticky="ew", padx=(6, 0))
        self.cal_gripper_lab = ttk.Label(lf)
        self.cal_gripper_lab.grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))
        self.cal_gripper_help = ttk.Label(lf, wraplength=420, justify="left", foreground="#374151",
                                          text=JOINT_HELP[4])
        self.cal_gripper_help.grid(row=2, column=0, columnspan=4, sticky="w", pady=(8, 0))

        # 3D view of the arm as the application understands it: it must move like the real arm
        self.cal_help_3d = ttk.Label(right, foreground="#b45309", wraplength=900, justify="left",
                                     text="The drawing below must move like the real arm when you click +10 / −10. "
                                     "If it turns the other way or stays offset, a point is wrong: delete it and "
                                     "do it again.")
        self.cal_help_3d.pack(side="top", fill="x", pady=(8, 0))
        self.view3d_cal = View3D(right, self)
        self.view3d_cal.configure(height=200)
        self.view3d_cal.zoom = 1.15
        self.view3d_cal.pack(side="top", fill="both", expand=True, pady=(4, 0))
        self._cal_select(send=False)

    def _geometry_tab(self, nb):
        f = ttk.Frame(nb, padding=14)
        nb.add(f, text="  Geometry  ")
        self.geo_vars = {}
        for k, (key, text) in enumerate(GEOMETRY_FIELDS):
            ttk.Label(f, text=text, style="Alert.TLabel" if key == "H0" else "TLabel").grid(
                row=k, column=0, sticky="w", pady=2)
            v = tk.StringVar()
            ttk.Entry(f, textvariable=v, width=10).grid(row=k, column=1, padx=8)
            ttk.Label(f, text="mm").grid(row=k, column=2, sticky="w")
            self.geo_vars[key] = v
        b = ttk.Frame(f)
        b.grid(row=len(GEOMETRY_FIELDS), column=0, columnspan=3, sticky="w", pady=(12, 0))
        ttk.Button(b, text="Apply", command=self._geo_apply).pack(side="left")
        ttk.Button(b, text="Back to the CAD values", command=self._geo_cad).pack(side="left", padx=8)
        ttk.Label(f, wraplength=780, justify="left", foreground="#374151", text=(
            "The dimensions come from the SolidWorks assembly (arm V2, new gripper, new base). H0 must be measured "
            "on the real robot: put a ruler on the table and measure the height of the centre of the shoulder servo "
            "axis (the centre of the horn). The CAD model gives about 97 mm with base V2, but the cover / horn stack "
            "is not certain in the assembly. An error on H0 shifts every height by the same amount.\n\nThe "
            "« Safety » values define the collisions refused by the application: increase them if the arm passes "
            "too close to the table or to the base.")).grid(
            row=len(GEOMETRY_FIELDS) + 1, column=0, columnspan=3, sticky="w", pady=(14, 0))
        self._geo_fill()

    # ------------------------------------------------------------------ helpers
    def message(self, text, kind="info"):
        colours = {"info": "#1f2937", "ok": "#166534", "error": "#b91c1c"}
        self.msg_lab.config(text=text, fg=colours.get(kind, "#1f2937"))

    def save(self):
        try:
            self.cfg.save(CONFIG_FILE)
        except Exception as e:
            self.message("Cannot save arm_config.json: %s" % e, "error")

    def phi_fixed(self):
        m = self.gripper_mode.get()
        if m == GRIPPER_MODES[1]:
            return 0.0
        if m == GRIPPER_MODES[2]:
            return -90.0
        if m == GRIPPER_MODES[3]:
            try:
                return number(self.gripper_angle.get())
            except ValueError:
                return None
        return None

    def q_real_full(self):
        return None if any(v is None for v in self.q_real) else tuple(self.q_real)

    def q_shown(self):
        """Robot position if known (servos holding), otherwise the command."""
        return self.q_real_full() or self.q_cmd

    def can_send(self):
        return self.link is not None and self.bridge_ok and self.allowed and self.active

    def path_ok(self, q):
        """Is the move to q safe, from the command and from the real robot position?"""
        starts = [self.q_cmd]
        qr = self.q_real_full()
        if qr is not None:
            starts.append(qr)
        for s in starts:
            ok, reason = path_is_safe(self.cfg.geometry, self.lim, s, q)
            if not ok:
                return False, reason
        return True, ""

    # ------------------------------------------------------------------ main loop (every 20 ms)
    def _tick(self):
        try:
            self._read_link()
            self._maps_computed()
            t = time.monotonic()
            if self.follow_target is not None and self.follow_var.get() and t - self._t["follow"] >= 0.04:
                self._t["follow"] = t
                c, self.follow_target = self.follow_target, None
                self._follow_to(c)
            if self._to_send and t - self._t["send"] >= 0.04 and self.can_send():
                self._t["send"] = t
                self._to_send = False
                self._send_command()
            if self._jog is not None and t - self._t["jog"] >= 0.04 and self.link is not None:
                self._t["jog"] = t
                channel, us = self._jog
                self._jog = None
                u = [0] * 5
                u[channel] = int(round(us))
                self.link.send("T " + " ".join(map(str, u)))
            if self.link is not None and t - self._t["ping"] >= 2.0:
                self._t["ping"] = t
                self.link.send("?")
            for name in ("top", "side") if self.tabs.index("current") == 1 else ():
                if self._images_to_redo[name] and t - self._t[name] >= 0.08:
                    self._t[name] = t
                    self._images_to_redo[name] = False
                    (self.top_map if name == "top" else self.side_map).draw(True)
            if self._to_redraw and t - self._t["draw"] >= 0.04:
                self._t["draw"] = t
                self._to_redraw = False
                self._redraw()
            self._update_status()
        except Exception as e:                       # an error must never stop the loop
            self.message("Internal error: %r" % (e,), "error")
        self.after(20, self._tick)

    def _redraw(self):
        tab = self.tabs.index("current")
        if tab == 0:
            self.view3d.draw()
            self._update_info()
        elif tab == 1:
            self._redraw_maps()
        elif tab == 2:
            self.view3d_cal.draw()
        self._update_sliders()

    def _tab_changed(self):
        self._images_to_redo = {"top": True, "side": True}
        self._to_redraw = True
        if self.tabs.index("current") == 2:
            self._cal_select(send=False)

    # ------------------------------------------------------------------ link
    def _list_ports(self):
        ports = list_ports()
        self.port_combo.config(values=ports)
        if self.cfg.port and self.cfg.port not in ports:
            name = self.cfg.port.split(" - ")[0]
            for p in ports:
                if p.split(" - ")[0] == name:
                    self.cfg.port = p

    def connect(self):
        name = self.port_var.get() or SIMULATION
        self.disconnect()
        try:
            self.link = SimulatedLink() if name == SIMULATION else SerialLink(name)
        except Exception as e:
            self.link = None
            port = name.split(" - ")[0]
            text = str(e)
            if "PermissionError" in text or "Accès refusé" in text or "Access is denied" in text:
                hint = ("Port %s is already open in another program: only one program at a time can use it.\n\n"
                        "Most of the time it is the Arduino IDE serial monitor: close its « Serial Monitor » tab "
                        "(or close the IDE), then click « Connect » again." % port)
            elif "FileNotFoundError" in text or "cannot find" in text.lower():
                hint = ("Port %s does not exist any more: is the ESP32 still plugged in? Click ↻ to refresh the "
                        "list of ports." % port)
            else:
                hint = "Cannot open %s." % port
            messagebox.showerror("Connection", "%s\n\n(details: %s)" % (hint, text))
            return
        self.bridge_ok = False
        self._first_pos = True
        self._t["answer"] = time.monotonic()
        self._t["ping"] = 0.0
        if name != SIMULATION:
            self.cfg.port = name
            self.save()
        self.message("Connecting to %s..." % name)

    def disconnect(self):
        if self.link is not None:
            self.link.close()
        self.link = None
        self.bridge_ok = self.active = self.allowed = False
        self.q_real, self.gripper_real, self.us_real = [None] * 4, None, None
        self.move_queue.clear()
        self._m_running = self._gripper_after_m = False
        self._to_redraw = True

    def _read_link(self):
        if self.link is None:
            return
        for _ in range(200):
            try:
                line = self.link.received.get_nowait()
            except queue.Empty:
                break
            self._line_received(line)

    def _line_received(self, line):
        self._t["answer"] = time.monotonic()
        if line.startswith("!disconnected"):
            self.disconnect()
            self.message("Link lost: %s" % line[14:], "error")
            return
        if line.startswith(("BRIDGE", "PONT")):          # "PONT": older French firmware, still accepted
            first = not self.bridge_ok
            self.bridge_ok = True
            m = re.search(r"pca=(\w+) err=(\d+)", line)
            if m:
                self.pca_state, self.i2c_errors = m.group(1), int(m.group(2))
            elif "error" in line or "erreur" in line:
                self.pca_state = "missing"
                self.message("ESP32: " + line, "error")
            if first:
                self._send_speed()
                self.message("Robot connected (arm_bridge firmware).", "ok")
        m = re.search(r"pos((?: -?\d+){5})\s*$", line)
        if m:
            self._position_received([int(v) for v in m.group(1).split()])
        elif line in ("done", "fini"):
            if self.move_queue:
                self._next_move()
            else:                           # end of the "Go" move: a gripper change asked meanwhile now follows
                self._m_running = False
                if self._gripper_after_m:
                    self._gripper_after_m = False
                    self.command_changed()
        elif line.startswith("err"):
            self.message("ESP32: " + line, "error")

    def _position_received(self, u):
        self.us_real = u
        q, p = self.cfg.from_pulses(u)
        self.q_real = [None if v is None else float(v) for v in q]
        self.gripper_real = p
        self.active = all(v is not None for v in q)
        if self._first_pos and self.bridge_ok:
            self._first_pos = False
            if self.active:      # the robot was already holding a posture (application restarted...)
                self.q_cmd = into_ranges(self.q_real, self.lim)
                if p is not None:
                    self.gripper_pct = p
                ok, reason, _ = posture_is_valid(self.cfg.geometry, self.lim, self.q_cmd)
                if ok:
                    self.allowed = True
                    self.message("The robot was already holding a posture: the command starts from there.", "ok")
                else:           # pulses left in the PCA9685 by an older program
                    self.message("The servos are holding a posture left by an older program (%s). Click « Activate "
                                 "servos »: the arm will go to the rest position at limited speed." % reason, "error")
        self._to_redraw = True

    def _update_status(self):
        if self.link is None:
            text, colour = "Disconnected", "#6b7280"
        else:
            name = "Simulation" if isinstance(self.link, SimulatedLink) else self.link.name
            if not self.bridge_ok:
                waiting = time.monotonic() - self._t["answer"]
                text = "%s: no answer from the arm_bridge firmware" % name if waiting > 3 else "%s: connecting..." % name
                colour = "#b45309"
            else:
                servos = ("servos active" if self.allowed else "servos holding, to activate") if self.active \
                    else "servos released"
                pca = "PCA9685 ok" if self.pca_state == "ok" else "PCA9685 NOT FOUND"
                text = "%s  •  %s  •  %s  •  I2C: %d err." % (name, pca, servos, self.i2c_errors)
                colour = "#166534" if (self.pca_state == "ok" and self.active and self.allowed) else "#b45309"
                if self.pca_state != "ok":
                    colour = "#b91c1c"
        if self.status_lab.cget("text") != text:
            self.status_lab.config(text=text, fg=colour)

    def _send_speed(self):
        if self.link is not None:
            v = self.cfg.speed * US_PER_DEG
            self.link.send("V %.0f %.0f" % (v, 4 * v))

    def _safe_pulses(self, q, pct):
        """Pulse widths for posture q, or None if one of them is outside its limits (should not happen)."""
        u = self.cfg.pulses(q, pct)
        for j in self.cfg.joints:
            if not j.us_min - 2 <= u[j.channel] <= j.us_max + 2:
                self.message("Refused: the pulse width of the « %s » servo (%d µs) is outside its limits." % (
                    j.name, u[j.channel]), "error")
                return None
        return u

    def _send_command(self):
        u = self._safe_pulses(self.q_cmd, self.gripper_pct)
        if u is not None:
            self.link.send("T " + " ".join(map(str, u)))

    def command_changed(self):
        self._to_send = True
        self._to_redraw = True
        self._m_running = self._gripper_after_m = False     # a T command replaces the "Go" move

    def toggle_gripper(self):
        """Opens the gripper if it is closed, closes it otherwise (button or space bar on the map)."""
        self.gripper_pct = 0.0 if self.gripper_pct >= 50 else 100.0
        state = "open" if self.gripper_pct >= 50 else "closed"
        if self._m_running:       # do not interrupt a "Go" move: the gripper will follow at the end
            self._gripper_after_m = True
            self.message("Gripper %s at the end of the move." % state)
        else:
            self._to_send = True
            self.message("Gripper %s.%s" % (state, "" if self.can_send() else
                                            "  (servos released: preview only)"), "ok")
        self._to_redraw = True

    def activate(self):
        if self.link is None or not self.bridge_ok:
            messagebox.showwarning("Activate", "The robot is not connected (or does not answer).")
            return
        g, lim = self.cfg.geometry, self.lim
        ok, reason, _ = posture_is_valid(g, lim, self.q_cmd)
        if not ok:
            rest = into_ranges(self.cfg.rest, lim)
            if not posture_is_valid(g, lim, rest)[0]:
                messagebox.showwarning("Activate", "The current command is not valid (%s), and neither is the rest "
                                       "position with the current limits.\n\nSet the sliders of the Control tab to "
                                       "a valid posture, then activate." % reason)
                return
            if not messagebox.askyesno("Activate", "The current command is not valid (%s).\n\n"
                                       "Use the rest position as the command?" % reason):
                return
            self.q_cmd = rest
            self.target = None
            self._to_redraw = True
        qr = self.q_real_full()
        if qr is not None:
            # the servos already hold a posture: they will go to the command at limited speed, without a jump
            ok, reason = path_is_safe(g, lim, qr, self.q_cmd)
            if not ok:
                messagebox.showwarning("Activate", "The servos are already holding a posture, and the move to the "
                                       "command would be dangerous (%s).\n\nPress RELEASE, put the arm by hand close "
                                       "to the grey posture, then activate." % reason)
                return
        elif not isinstance(self.link, SimulatedLink) and not messagebox.askokcancel(
                "Activate servos",
                "The servos will jump at once (no ramp) to the posture drawn in grey in the 3D view.\n\n"
                "First put the arm by hand close to this posture, and keep your hand near the RELEASE button "
                "(Esc key).\n\nContinue?"):
            return
        u = self._safe_pulses(self.q_cmd, self.gripper_pct)
        if u is None:
            return
        self.move_queue.clear()
        self.allowed = True
        self.link.send("T " + " ".join(map(str, u)))
        self.message("Servos activated%s." % (": the arm joins the command at limited speed" if qr is not None else ""),
                     "ok")

    def release(self):
        self.allowed = False
        self._to_send = False
        self._jog = None
        self.move_queue.clear()
        self._m_running = self._gripper_after_m = False
        if self.link is not None:
            self.link.send("O")
        self.message("Servos released: the arm is no longer held. « Activate servos » to resume.", "error")

    def quit_app(self):
        self.save()
        if self.link is not None:
            self.link.close()
        self.destroy()

    # ------------------------------------------------------------------ moves
    def move_to(self, q_goal):
        """Synchronised move (M command) to posture q_goal, along a checked path. If the direct path would hit the
        table or the base, the arm goes through the rest position."""
        g, lim = self.cfg.geometry, self.lim
        if not self.can_send():           # servos released: only the command moves (preview)
            self.move_queue.clear()
            self.q_cmd = into_ranges(q_goal, lim)
            self._to_send = False
            self._to_redraw = True
            return True
        q0 = self.q_shown()
        ok, reason = path_is_safe(g, lim, q0, q_goal)
        steps = [q_goal] if ok else None
        if not ok:
            rest = into_ranges(self.cfg.rest, lim)
            if posture_is_valid(g, lim, rest)[0] and path_is_safe(g, lim, q0, rest)[0] and \
                    path_is_safe(g, lim, rest, q_goal)[0]:
                steps = [rest, q_goal]
        if steps is None:
            self.message("Move refused: %s (and no safe detour through the rest position)." % reason, "error")
            return False
        self.move_queue = [into_ranges(q, lim) for q in steps]
        self._next_move()
        if len(steps) > 1:
            self.message("The direct path would hit something: going through the rest position.", "info")
        return True

    def _next_move(self):
        if not self.move_queue:
            return
        self._to_send = False
        self._to_redraw = True
        if not self.can_send():          # no active robot: preview of the final position
            self.q_cmd = self.move_queue[-1]
            self.move_queue.clear()
            return
        q = self.move_queue.pop(0)
        travel = max(abs(a - b) for a, b in zip(q, self.q_shown()))
        duration = max(300.0, 1500.0 * travel / max(5.0, self.cfg.speed))   # peak speed = 1.5 x the mean speed
        u = self._safe_pulses(q, self.gripper_pct)
        if u is None:
            self.move_queue.clear()
            return
        self.q_cmd = q
        self.link.send("M %d %s" % (duration, " ".join(map(str, u))))
        self._m_running = True

    def go_to_position(self, X, Y, Z):
        g = self.cfg.geometry
        s = solve(g, self.lim, X, Y, Z, q_ref=self.q_shown(), phi_fixed=self.phi_fixed())
        self.target = np.array([X, Y, Z], dtype=float)
        self.target_ok = s.ok
        if not s.ok:
            text = "Position (%.0f, %.0f, %.0f) impossible: %s." % (X, Y, Z, s.reason)
            reach_map = self.current_map()
            if reach_map is not None:
                p = reach_map.nearest(X, Y, Z)
                if p is not None:
                    text += "  Nearest reachable: (%.0f, %.0f, %.0f)." % p
            self.message(text, "error")
            self._to_redraw = True
            return False
        if self.move_to(s.q):
            self.message("Moving to (%.0f, %.0f, %.0f): gripper at %.1f°, margin to the limits %.1f°%s." % (
                X, Y, Z, s.phi, s.margin, "" if s.side > 0 else ", over the shoulder"), "ok")
            return True
        return False

    def go_to_coordinates(self):
        try:
            X, Y, Z = (number(v.get()) for v in self.coord)
        except ValueError:
            self.message("Invalid coordinates.", "error")
            return
        self.go_to_position(X, Y, Z)

    def _current_coords(self):
        P, _ = fk(self.cfg.geometry, self.q_shown())
        for v, x in zip(self.coord, P):
            v.set("%.1f" % x)

    def go_rest(self):
        q = into_ranges(self.cfg.rest, self.lim)
        if not posture_is_valid(self.cfg.geometry, self.lim, q)[0]:
            self.message("The saved rest position is not valid with the current limits.", "error")
            return
        self.target = None
        if self.move_to(q):
            self.message("Going back to the rest position.", "ok")

    def store_rest(self):
        self.cfg.rest = [round(v, 2) for v in self.q_cmd]
        self.save()
        self.message("Rest position saved: %s" % ", ".join("%.1f°" % v for v in self.q_cmd), "ok")

    # ------------------------------------------------------------------ sliders
    def _update_sliders(self):
        self._auto_update = True
        try:
            for i, s in enumerate(self.sliders):
                if abs(float(s.get()) - self.q_cmd[i]) > 0.05:
                    s.set(self.q_cmd[i])
                self.slider_values[i].config(text="%.1f°" % self.q_cmd[i])
            if abs(float(self.gripper_slider.get()) - self.gripper_pct) > 0.05:
                self.gripper_slider.set(self.gripper_pct)
            self.gripper_value.config(text="%.0f %%" % self.gripper_pct)
            if abs(float(self.speed_slider.get()) - self.cfg.speed) > 0.05:
                self.speed_slider.set(self.cfg.speed)
            self.speed_value.config(text="%.0f °/s" % self.cfg.speed)
        finally:
            self._auto_update = False

    def _slider(self, i, value):
        if self._auto_update:
            return
        q = list(self.q_cmd)
        q[i] = float(value)
        if self.can_send():          # servos released: only the command moves, nothing to check
            ok, reason = self.path_ok(q)
            if not ok:
                self.message("%s: %s, move refused." % (DISPLAY_NAMES[i], reason), "error")
                self._to_redraw = True       # the slider goes back to the command
                return
        self.move_queue.clear()
        self.q_cmd = tuple(q)
        self.target = None
        self.command_changed()

    def _gripper_slider(self, value):
        if self._auto_update:
            return
        self.gripper_pct = float(value)
        self.command_changed()

    def _speed(self, value):
        if self._auto_update:
            return
        self.cfg.speed = round(float(value))
        self.speed_value.config(text="%.0f °/s" % self.cfg.speed)
        self._send_speed()
        self.save()

    def _gripper_mode_changed(self):
        self.follow_target = None
        self._request_map()
        self._images_to_redo = {"top": True, "side": True}
        self._to_redraw = True

    def _update_info(self):
        g = self.cfg.geometry
        P, phi = fk(g, self.q_cmd)
        ok, reason, margin = posture_is_valid(g, self.lim, self.q_cmd)
        u = self.cfg.pulses(self.q_cmd, self.gripper_pct)
        lines = ["Command   X %6.1f  Y %6.1f  Z %6.1f mm" % tuple(P),
                 "          gripper %+.1f°, " % phi + (("margin %.1f°" % margin) if ok else reason.upper())]
        qr = self.q_real_full()
        if qr is not None:
            Pr, _ = fk(g, qr)
            lines.append("Robot     X %6.1f  Y %6.1f  Z %6.1f mm" % tuple(Pr))
        lines.append("µs (channel): " + "  ".join("%s %d (%d)" % (n[:3].lower(), u[j.channel], j.channel) for j, n in
                                                  zip(self.cfg.joints, DISPLAY_NAMES[:2])))
        lines.append("              " + "  ".join("%s %d (%d)" % (n[:3].lower(), u[j.channel], j.channel) for j, n in
                                                  zip(self.cfg.joints[2:], DISPLAY_NAMES[2:4])) +
                     "  gri %d (%d)" % (u[self.cfg.gripper.channel], self.cfg.gripper.channel))
        todo = [n for j, n in zip(self.cfg.joints, DISPLAY_NAMES) if not j.is_calibrated()]
        if todo:
            lines.append("To calibrate: " + ", ".join(todo))
        text = "\n".join(lines)
        if self.control_info.cget("text") != text:
            self.control_info.config(text=text)

    # ------------------------------------------------------------------ map
    def _map_key(self):
        phi = self.phi_fixed()
        return (None if phi is None else round(phi, 1), tuple(sorted(self.cfg.geometry.to_dict().items())),
                tuple(np.round(self.lim, 2).ravel().tolist()))

    def current_map(self):
        return self.maps.get(self._map_key())

    def _request_map(self):
        key = self._map_key()
        if key in self.maps or key in self._computing:
            return
        self._computing.add(key)
        g = Geometry.from_dict(self.cfg.geometry.to_dict())
        lim, phi = self.lim.copy(), self.phi_fixed()

        def compute():
            try:
                c = ReachMap(g, lim, phi_fixed=phi)
            except Exception as e:
                c = e
            self._maps_ready.put((key, c))
        threading.Thread(target=compute, daemon=True).start()

    def _maps_computed(self):
        while True:
            try:
                key, c = self._maps_ready.get_nowait()
            except queue.Empty:
                return
            self._computing.discard(key)
            if isinstance(c, Exception):
                self.message("Cannot compute the map: %r" % (c,), "error")
                continue
            self.maps[key] = c
            while len(self.maps) > 6:
                self.maps.pop(next(iter(self.maps)))
            if key == self._map_key():
                self._images_to_redo = {"top": True, "side": True}
                self._to_redraw = True

    def _redraw_maps(self):
        self.top_title.config(text="Top view — section at height Y = %.0f mm (wheel)" % self.map_height)
        self.side_title.config(text="Side view — direction %+.0f° (wheel)" % (self.azimuth / DEG))
        self.top_map.draw()
        self.side_map.draw()
        g = self.cfg.geometry
        P, phi = fk(g, self.q_shown())
        text = "Robot: X %6.1f  Y %6.1f  Z %6.1f  gripper %+5.1f° %s" % (
            P[0], P[1], P[2], phi, "open" if self.gripper_pct >= 50 else "closed")
        if self.target is not None:
            text += "     Target: X %6.1f  Y %6.1f  Z %6.1f  %s" % (
                self.target[0], self.target[1], self.target[2], "reachable" if self.target_ok else "OUT OF REACH")
        self.map_info.config(text=text)
        self.follow_info_lab.config(text=self.follow_info)
        gripper_text = "Close the gripper (Space)" if self.gripper_pct >= 50 else "Open the gripper (Space)"
        if self.gripper_button.cget("text") != gripper_text:
            self.gripper_button.config(text=gripper_text)

    def mouse_on_map(self, view, u, v):
        X, Y, Z = view.point3d(u, v)
        if view.kind == "side":
            if abs(Y - self.map_height) >= 2:
                self.map_height = Y
                self._images_to_redo["top"] = True
        elif math.hypot(X, Z) > 25:
            az = math.atan2(Z, X)
            if abs(math.remainder(az - self.azimuth, 2 * math.pi)) >= DEG:
                self.azimuth = az
                self._images_to_redo["side"] = True
        self.mouse_view = view
        if self.follow_var.get():
            self.follow_target = (X, Y, Z)
        self._to_redraw = True

    def mouse_left_map(self, view):
        self.mouse_view = None       # the robot finishes going to the last mouse position

    def map_click(self, view):
        self.follow_var.set(not self.follow_var.get())
        self.message("Mouse following resumed." if self.follow_var.get() else
                     "Mouse following paused: the mouse can move without moving the robot.")
        self._to_redraw = True

    def map_go(self, view, u, v):
        X, Y, Z = view.point3d(u, v)
        self.follow_var.set(False)
        self.go_to_position(X, Y, Z)
        self.follow_info = "following paused during the move (left-click to resume)"

    def map_wheel(self, view, delta):
        if view.kind == "top":
            g = self.cfg.geometry
            self.map_height = min(g.H0 + g.reach, max(0.0, self.map_height + (5 if delta > 0 else -5)))
            self._images_to_redo["top"] = True
            if self.follow_var.get() and self.target is not None:
                self.follow_target = (self.target[0], self.map_height, self.target[2])
        else:
            self.azimuth += (3 if delta > 0 else -3) * DEG
            self._images_to_redo["side"] = True
        self._to_redraw = True

    def _follow_to(self, target):
        """The robot follows the mouse: continuous path (no change of posture) to the aimed point, or to the nearest
        reachable point in the same direction."""
        g, lim = self.cfg.geometry, self.lim
        reach_map = self.current_map()
        self.target_ok = True
        mouse_target = target
        if reach_map is not None:
            ok, _ = reach_map.point(*target)
            if not bool(ok):
                self.target_ok = False
                nearest = reach_map.nearest(*target)
                if nearest is None:
                    self.target = np.array(target)
                    self.follow_info = "nothing reachable in this direction"
                    self._to_redraw = True
                    return
                target = nearest
        self.target = np.array(target, dtype=float)
        postures, reason = follow_path(g, lim, self.q_cmd, target, phi_fixed=self.phi_fixed())
        starts = [self.q_cmd]
        qr = self.q_real_full()
        if qr is not None:
            starts.append(qr)
        # the servos move in a straight line (in joint space) from their position to the command: send the furthest
        # point of the path for which that motion is safe; the rest follows at the next tick
        for k in range(len(postures) - 1, -1, -1):
            if all(path_is_safe(g, lim, s, postures[k])[0] for s in starts):
                self.q_cmd = postures[k]
                self.move_queue.clear()
                self.command_changed()
                if k < len(postures) - 1 and self.follow_target is None:
                    self.follow_target = mouse_target
                break
        else:
            reason = reason or "move refused (collision)"
        self.follow_info = reason if reason else ("" if self.target_ok else "point out of reach: going to the nearest")
        if not self.can_send() and self.link is not None and self.bridge_ok:
            self.follow_info += ("  " if self.follow_info else "") + "[servos released: preview only]"
        self._to_redraw = True

    # ------------------------------------------------------------------ calibration
    def _cal_channel_of(self, j):
        return self.cfg.joints[j].channel if j < 4 else self.cfg.gripper.channel

    def _cal_select(self, send=False):
        j = self.cal_j.get()
        channel = self._cal_channel_of(j)
        self.cal_channel.set(str(channel))
        if self.us_real and self.us_real[channel] > 0:
            us = self.us_real[channel]
        elif j < 4:
            us = self.cfg.joints[j].to_us(self.q_cmd[j])
        else:
            us = self.cfg.gripper.to_us(self.gripper_pct)
        self._cal_set(us, send=send)
        if j < 4:
            self.cal_gripper_frame.pack_forget()
            self.cal_joint_frame.pack(fill="x")
            self.cal_right_frame.pack(side="top", fill="x", before=self.cal_help_3d)
            jt = self.cfg.joints[j]
            self.cal_zero.set("%.1f" % jt.zero_angle)
            self.cal_direction.set("%+d" % jt.direction)
            for b, (angle, text) in zip(self.cal_ref_buttons, REFERENCES[j]):
                b.config(text=text)
            self.cal_help_lab.config(text=JOINT_HELP[j])
        else:
            self.cal_joint_frame.pack_forget()
            self.cal_right_frame.pack_forget()
            self.cal_gripper_frame.pack(fill="x")
        self._to_redraw = True
        self._cal_texts()

    def _cal_slider(self, value):
        if not self._auto_update:
            self._cal_set(float(value))

    def _cal_set(self, us, send=True):
        us = float(min(2500.0, max(500.0, round(us))))
        self.cal_us = us
        self._auto_update = True
        try:
            self.cal_slider.set(us)
        finally:
            self._auto_update = False
        if send and self.cal_send.get() and self.link is not None:
            j = self.cal_j.get()
            self._jog = (self._cal_channel_of(j), us)
            if j < 4:          # the command follows, so that the Control tab starts from the right position
                q = list(self.q_cmd)
                q[j] = self.cfg.joints[j].to_angle(us)
                self.q_cmd = tuple(q)
            else:
                self.gripper_pct = self.cfg.gripper.to_pct(us)
            self._to_redraw = True
        self._cal_texts()

    def _cal_texts(self):
        j, us = self.cal_j.get(), self.cal_us
        self.cal_us_lab.config(text="%d µs" % us)
        if j == 4:
            gr = self.cfg.gripper
            self.cal_angle_lab.config(text="opening ≈ %.0f %%" % gr.to_pct(us))
            self.cal_gripper_lab.config(text="open: %d µs     closed: %d µs" % (gr.us_open, gr.us_closed))
            return
        jt = self.cfg.joints[j]
        self.cal_angle_lab.config(text="≈ %.1f° with the current calibration%s" % (
            jt.to_angle(us), "" if jt.is_calibrated() else " (not calibrated: rough value)"))
        self.cal_limits_lab.config(text="low: %d µs (%.1f°)      high: %d µs (%.1f°)" % (
            jt.us_min, jt.to_angle(jt.us_min), jt.us_max, jt.to_angle(jt.us_max)))
        self.cal_list.delete(*self.cal_list.get_children())
        pts = sorted((float(u), float(v)) for u, v in jt.points)
        deviations = [""] * len(pts)
        if len(pts) >= 3:
            x = np.array([p[0] for p in pts])
            y = np.array([p[1] for p in pts])
            pa, pb = np.polyfit(x, y, 1)
            deviations = ["%+.2f" % e for e in y - (pa * x + pb)]
        for (u, v), e in zip(pts, deviations):
            self.cal_list.insert("", "end", values=("%.0f" % u, "%.2f" % v, e))
        self.cal_summary.config(text=jt.summary(), foreground="#b91c1c" if jt.problem() else "#374151")

    def _calibration_changed(self):
        """After a calibration change of the selected joint: the command keeps the same pulse width (the one set with
        +10 / −10, even if the servo has not reached it yet), only the matching angle changes."""
        j = self.cal_j.get()
        if j < 4:
            q = list(self.q_cmd)
            q[j] = self.cfg.joints[j].to_angle(self.cal_us)
            self.q_cmd = tuple(q)
        if self.us_real:
            self._position_received(self.us_real)      # robot position read again with the new calibration
        self._limits_changed()
        self._cal_texts()

    def _limits_changed(self, save=True):
        self.lim = self.cfg.limits()
        self._auto_update = True
        try:
            for i, s in enumerate(self.sliders):
                s.config(from_=self.lim[i][0], to=self.lim[i][1])
        finally:
            self._auto_update = False
        self._request_map()
        self._images_to_redo = {"top": True, "side": True}
        self._to_redraw = True
        if save:
            self.save()

    def _cal_channel_changed(self):
        j = self.cal_j.get()
        try:
            new = int(self.cal_channel.get())
        except ValueError:
            return
        old = self._cal_channel_of(j)
        if new == old:
            return
        for k in range(5):            # the servo that had this channel takes the old channel (swap)
            if k != j and self._cal_channel_of(k) == new:
                if k < 4:
                    self.cfg.joints[k].channel = old
                else:
                    self.cfg.gripper.channel = old
                self.message("Channels swapped: %s → channel %d, %s → channel %d." % (
                    DISPLAY_NAMES[j], new, DISPLAY_NAMES[k], old), "info")
        if j < 4:
            self.cfg.joints[j].channel = new
        else:
            self.cfg.gripper.channel = new
        self.save()
        if self.us_real:
            self._position_received(self.us_real)
        self._cal_texts()

    def _cal_limit(self, k):
        jt = self.cfg.joints[self.cal_j.get()]
        if k == 0:
            jt.us_min = self.cal_us
        else:
            jt.us_max = self.cal_us
        if jt.us_min > jt.us_max:
            jt.us_min, jt.us_max = jt.us_max, jt.us_min
        if jt.us_max - jt.us_min < 50:
            self.message("Both limits are almost at the same place: check them.", "error")
        self._calibration_changed()
        self.message("Limit saved: %s from %d to %d µs." % (jt.name, jt.us_min, jt.us_max), "ok")

    def _cal_go_limit(self, k):
        jt = self.cfg.joints[self.cal_j.get()]
        self._cal_set(jt.us_min if k == 0 else jt.us_max)

    def _cal_add_ref(self, k):
        angle, text = REFERENCES[self.cal_j.get()][k]
        self._cal_add(angle, text.split(": ", 1)[1])

    def _cal_add(self, angle=None, description=""):
        j = self.cal_j.get()
        jt = self.cfg.joints[j]
        if angle is None:
            try:
                angle = number(self.cal_measure.get())
            except ValueError:
                self.message("Type the measured real angle (in degrees), or use a position button.", "error")
                return
            self.cal_measure.set("")
        jt.points = [p for p in jt.points if abs(float(p[0]) - self.cal_us) > 0.5] + [[self.cal_us, float(angle)]]
        self._calibration_changed()
        pb = jt.problem()
        if pb:
            self.message("Warning, %s: %s." % (jt.name, pb), "error")
        else:
            self.message("Point added: %s, %d µs = %+.0f°%s." % (
                DISPLAY_NAMES[j], self.cal_us, angle, (" (" + description + ")") if description else ""), "ok")

    def _cal_selection(self):
        sel = self.cal_list.selection()
        if not sel:
            self.message("Select a point in the list.", "error")
            return None
        return float(self.cal_list.item(sel[0], "values")[0])

    def _cal_delete(self):
        us = self._cal_selection()
        if us is None:
            return
        jt = self.cfg.joints[self.cal_j.get()]
        jt.points = [p for p in jt.points if abs(float(p[0]) - us) > 0.5]
        self._calibration_changed()

    def _cal_go_point(self):
        us = self._cal_selection()
        if us is not None:
            self._cal_set(us)

    def _cal_go_angle(self):
        jt = self.cfg.joints[self.cal_j.get()]
        try:
            angle = number(self.cal_test.get())
        except ValueError:
            self.message("Invalid angle.", "error")
            return
        us = jt.to_us(angle)
        if not jt.us_min - 1 <= us <= jt.us_max + 1:
            self.message("%.1f° needs %d µs: outside the limits (%d to %d µs)." % (angle, us, jt.us_min, jt.us_max),
                         "error")
            return
        self._cal_set(us)

    def _cal_rough(self):
        jt = self.cfg.joints[self.cal_j.get()]
        try:
            jt.zero_angle = number(self.cal_zero.get())
            jt.direction = -1 if self.cal_direction.get().strip() == "-1" else 1
            jt.zero_us = 1500.0
        except ValueError:
            self.message("Invalid angle.", "error")
            return
        self._calibration_changed()
        self.message("Rough setting saved (used while there are fewer than 2 points).", "ok")

    def _cal_gripper(self, k):
        gr = self.cfg.gripper
        if k == 0:
            gr.us_open = self.cal_us
        else:
            gr.us_closed = self.cal_us
        self.save()
        self._cal_texts()
        self.message("Gripper %s: %d µs." % ("open" if k == 0 else "closed", self.cal_us), "ok")

    # ------------------------------------------------------------------ geometry
    def _geo_fill(self):
        for key, v in self.geo_vars.items():
            v.set("%g" % getattr(self.cfg.geometry, key))

    def _geo_apply(self):
        try:
            d = {key: number(v.get()) for key, v in self.geo_vars.items()}
        except ValueError:
            self.message("A geometry value is not a number.", "error")
            return
        if min(d["L1"], d["L2"], d["L3"], d["H0"]) <= 0:
            self.message("L1, L2, L3 and H0 must be positive.", "error")
            return
        self.cfg.geometry = Geometry.from_dict(d)
        self.save()
        self._limits_changed(save=False)
        self.message("Geometry applied: the map of positions is computed again.", "ok")

    def _geo_cad(self):
        self.cfg.geometry = Geometry()
        self._geo_fill()
        self.save()
        self._limits_changed(save=False)
        self.message("CAD values restored.", "ok")


def main():
    try:                                   # sharp text on high-resolution screens (Windows)
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    App().mainloop()


if __name__ == "__main__":
    main()
