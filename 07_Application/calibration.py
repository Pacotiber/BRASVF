"""Servo calibration and the arm configuration file (arm_config.json).

Each joint has a table of measured points (pulse width in microseconds -> real angle in degrees, in the model
convention). With 2 points the mapping is a straight line; with 3 points or more it follows the points exactly
(piecewise-linear interpolation). This corrects the fact that "90 deg asked to the servo" is not 90 deg in real
life, nor a constant offset over the whole travel.
Limits are stored in microseconds and converted to angles with this calibration.
"""
import json
import os
from dataclasses import dataclass, field, asdict

import numpy as np

from kinematics import Geometry, JOINT_NAMES

US_PER_DEG = 2000.0 / 180.0     # MG995: about 500 to 2500 us for 180 deg (value used before calibration)
DISPLAY_NAMES = ("Base", "Shoulder", "Elbow", "Wrist", "Gripper")


def _interp(x, xs, ys):
    """Piecewise-linear interpolation, extended with the slope of the end segments (xs increasing)."""
    x = float(x)
    if x <= xs[0]:
        i = 0
    elif x >= xs[-1]:
        i = len(xs) - 2
    else:
        i = int(np.searchsorted(xs, x)) - 1
    return float(ys[i] + (x - xs[i]) * (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]))


@dataclass
class Joint:
    name: str
    channel: int
    zero_us: float = 1500.0       # without calibration: this pulse gives the angle zero_angle...
    zero_angle: float = 0.0
    direction: int = 1            # ...and the angle increases (+1) or decreases (-1) when the pulse increases
    us_min: float = 600.0         # limits (us), measured in the Calibration tab
    us_max: float = 2400.0
    points: list = field(default_factory=list)   # [[us, real angle], ...]

    def table(self):
        """(increasing pulses, angles) if the calibration is usable, otherwise None."""
        if len(self.points) < 2:
            return None
        p = sorted((float(u), float(a)) for u, a in self.points)
        us = np.array([u for u, _ in p])
        an = np.array([a for _, a in p])
        d = np.diff(an)
        if np.any(np.diff(us) <= 0) or not (np.all(d > 0) or np.all(d < 0)):
            return None
        return us, an

    def problem(self):
        """Text describing why the calibration points are unusable, otherwise ''."""
        if len(self.points) < 2:
            return ""
        p = sorted((float(u), float(a)) for u, a in self.points)
        if any(p[i + 1][0] == p[i][0] for i in range(len(p) - 1)):
            return "two points have the same pulse width"
        if self.table() is None:
            return "the angles do not always change in the same direction (is one point wrong?)"
        return ""

    def is_calibrated(self):
        return self.table() is not None

    def _reference(self):
        """Without a full calibration: the measured point if there is exactly one (fixes the offset), otherwise
        zero_us = zero_angle. The slope stays the default one, with the chosen direction."""
        if len(self.points) == 1:
            return float(self.points[0][0]), float(self.points[0][1])
        return self.zero_us, self.zero_angle

    def to_us(self, angle):
        t = self.table()
        if t is None:
            u0, a0 = self._reference()
            return u0 + self.direction * (float(angle) - a0) * US_PER_DEG
        us, an = t
        if an[0] > an[-1]:
            us, an = us[::-1], an[::-1]
        return _interp(angle, an, us)

    def to_angle(self, us_val):
        t = self.table()
        if t is None:
            u0, a0 = self._reference()
            return a0 + self.direction * (float(us_val) - u0) / US_PER_DEG
        return _interp(us_val, *t)

    def limits(self):
        a, b = self.to_angle(self.us_min), self.to_angle(self.us_max)
        return (min(a, b), max(a, b))

    def summary(self):
        """Calibration summary: mean slope and deviation of the points from a straight line."""
        t = self.table()
        if t is None:
            pb = self.problem()
            if pb:
                return "NOT CALIBRATED: " + pb
            if len(self.points) == 1:
                return ("1 point (%.0f µs = %.1f°): offset corrected, direction %+d assumed. Add a 2nd point far from "
                        "the first one to measure the direction and the slope." % (
                            self.points[0][0], self.points[0][1], self.direction))
            return "not calibrated: %.0f µs = %.0f°, direction %+d (default values, to be measured)" % (
                self.zero_us, self.zero_angle, self.direction)
        us, an = t
        a, b = np.polyfit(us, an, 1)
        deviation = float(np.max(np.abs(an - (a * us + b))))
        if len(us) == 2:
            return "calibrated (2 points, straight line): %.2f µs/°" % (1 / a)
        return "calibrated (%d points): %.2f µs/° on average, deviation from a straight line %.1f°" % (
            len(us), 1 / a, deviation)


@dataclass
class Gripper:
    channel: int = 4
    us_open: float = 1100.0
    us_closed: float = 1900.0

    def to_us(self, pct):
        return self.us_closed + (self.us_open - self.us_closed) * float(pct) / 100.0

    def to_pct(self, us_val):
        if self.us_open == self.us_closed:
            return 0.0
        return 100.0 * (float(us_val) - self.us_closed) / (self.us_open - self.us_closed)


def default_joints():
    # angle of each joint at 1500 us before calibration (to be corrected in the Calibration tab)
    return [Joint("base", 0, zero_angle=0.0),
            Joint("shoulder", 1, zero_angle=90.0),
            Joint("elbow", 2, zero_angle=-90.0),
            Joint("wrist", 3, zero_angle=0.0)]


# older French configuration file (config_bras.json): key names, read once and converted
_OLD_GEOMETRY = {"c_epaule": "lat_shoulder", "c_coude": "lat_elbow", "c_pince": "lat_wrist",
                 "marge_table": "table_margin", "rayon_servo": "servo_radius", "base_rayon": "base_radius",
                 "base_hauteur": "base_height", "pince_long_min": "gripper_len_min",
                 "pince_perp_min": "gripper_perp_min", "pince_perp_max": "gripper_perp_max",
                 "ouverture_max": "max_opening"}
_OLD_JOINT = {"nom": "name", "canal": "channel", "sens": "direction"}
_OLD_NAMES = {"epaule": "shoulder", "coude": "elbow", "poignet": "wrist"}
_OLD_GRIPPER = {"canal": "channel", "us_ouverte": "us_open", "us_fermee": "us_closed"}


def _from_old_format(d):
    """Converts the older French configuration file to the current key names."""
    j = []
    for a in d.get("articulations") or []:
        a = {_OLD_JOINT.get(k, k): v for k, v in a.items()}
        a["name"] = _OLD_NAMES.get(a.get("name"), a.get("name"))
        j.append(a)
    return {"geometry": {_OLD_GEOMETRY.get(k, k): v for k, v in (d.get("geometrie") or {}).items()},
            "joints": j,
            "gripper": {_OLD_GRIPPER.get(k, k): v for k, v in (d.get("pince") or {}).items()},
            "rest": d.get("repos"), "speed": d.get("vitesse", 60.0), "port": d.get("port", "")}


@dataclass
class Config:
    geometry: Geometry = field(default_factory=Geometry)
    joints: list = field(default_factory=default_joints)
    gripper: Gripper = field(default_factory=Gripper)
    rest: list = field(default_factory=lambda: [0.0, 90.0, -90.0, -30.0])
    speed: float = 60.0           # deg/s
    port: str = ""

    def limits(self):
        return np.array([j.limits() for j in self.joints], dtype=float)

    def channels(self):
        return [j.channel for j in self.joints] + [self.gripper.channel]

    def problems(self):
        pb = []
        c = self.channels()
        if len(set(c)) != len(c):
            pb.append("two servos are on the same PCA9685 channel")
        if any(not 0 <= k <= 4 for k in c):
            pb.append("channels must be between 0 and 4")
        for j in self.joints:
            if j.problem():
                pb.append("%s: %s" % (j.name, j.problem()))
        return pb

    def pulses(self, q=None, gripper_pct=None):
        """List of the 5 pulse widths per channel for posture q and the gripper opening (0 = unchanged)."""
        u = [0] * 5
        if q is not None:
            for j, v in zip(self.joints, q):
                u[j.channel] = int(round(j.to_us(v)))
        if gripper_pct is not None:
            u[self.gripper.channel] = int(round(self.gripper.to_us(gripper_pct)))
        return u

    def from_pulses(self, u):
        """Angles (None if the servo is released) and gripper opening from the pulse widths per channel."""
        q = [j.to_angle(u[j.channel]) if u[j.channel] > 0 else None for j in self.joints]
        p = self.gripper.to_pct(u[self.gripper.channel]) if u[self.gripper.channel] > 0 else None
        return q, p

    # ---------------------------------------------------------------- file
    def to_dict(self):
        return {"geometry": self.geometry.to_dict(),
                "joints": [asdict(j) for j in self.joints],
                "gripper": asdict(self.gripper), "rest": list(self.rest),
                "speed": self.speed, "port": self.port}

    def save(self, path):
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)
        os.replace(tmp, path)

    @classmethod
    def load(cls, path, old_path=None):
        """Reads the configuration. If it does not exist yet but the older French file (old_path) does, that one
        is converted, so an existing calibration is kept."""
        cfg = cls()
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                d = json.load(f)
        elif old_path and os.path.exists(old_path):
            with open(old_path, encoding="utf-8") as f:
                d = _from_old_format(json.load(f))
        else:
            return cfg
        cfg.geometry = Geometry.from_dict(d.get("geometry"))
        joints = d.get("joints") or []
        if len(joints) == 4:
            cfg.joints = [Joint(**{k: v for k, v in j.items() if k in Joint.__dataclass_fields__}) for j in joints]
        if d.get("gripper"):
            cfg.gripper = Gripper(**{k: v for k, v in d["gripper"].items() if k in Gripper.__dataclass_fields__})
        if len(d.get("rest") or []) == 4:
            cfg.rest = [float(v) for v in d["rest"]]
        cfg.speed = float(d.get("speed", cfg.speed))
        cfg.port = str(d.get("port", ""))
        return cfg
