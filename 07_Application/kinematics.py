"""Arm kinematics: same model as the MATLAB code and the ESP32 firmware (arm_ik), with an improved inverse
kinematics solver and a map of every reachable position.

Frame: X = forward, Y = up, Z = side (to the right when the base angle is 0).
Origin: on the table, on the rotation axis of the base. Units: mm and degrees.

Joint angles (degrees):
  base     : rotation about the vertical axis, positive towards +Z
  shoulder : arm 1 relative to the horizontal, positive upwards
  elbow    : arm 2 relative to the extension of arm 1
  wrist    : gripper axis relative to the extension of arm 2
  phi = shoulder + elbow + wrist = gripper pitch relative to the horizontal
"""
import math
from dataclasses import dataclass, asdict, fields

import numpy as np

DEG = math.pi / 180.0
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist")


@dataclass
class Geometry:
    # dimensions measured in the CAD model (Bras_robotiqueV2gripper + base V2), in mm
    L1: float = 153.86       # shoulder axis -> elbow axis
    L2: float = 120.39       # elbow axis -> wrist axis
    L3: float = 124.86       # wrist axis -> fingertips, along the gripper axis
    HP: float = 1.50         # fingertips offset perpendicular to the gripper axis
    R0: float = 10.25        # shoulder axis in front of the base axis
    H0: float = 97.40        # height of the shoulder axis above the table (base V2: measure it on the robot)
    lat_shoulder: float = 7.25   # sideways offsets (towards +Z) at each motor
    lat_elbow: float = 7.25
    lat_wrist: float = 7.25
    # safety
    table_margin: float = 5.0    # no point of the arm below this height (mm)
    servo_radius: float = 22.0   # size of the elbow and wrist servos around their axis
    base_radius: float = 60.0    # forbidden zone around the base axis...
    base_height: float = 65.0    # ...below this height
    # gripper outline in its own frame (along its axis / perpendicular)
    gripper_len_min: float = 20.0
    gripper_perp_min: float = -17.0
    gripper_perp_max: float = 19.0
    max_opening: float = 45.0    # finger gap when the gripper is open (drawing only)

    @property
    def C(self):
        """Total sideways offset."""
        return self.lat_shoulder + self.lat_elbow + self.lat_wrist

    @property
    def reach(self):
        return self.R0 + self.L1 + self.L2 + math.hypot(self.L3, self.HP)

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        known = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in (d or {}).items() if k in known})


def wrap180(a):
    """Brings an angle in degrees (or an array) between -180 and 180."""
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def into_range(a, lo, hi):
    """Writes the angle a (degrees), modulo 360, as close as possible to the middle of [lo, hi]: if the servo can
    reach the angle, this is the value that falls inside its range (useful when a range goes beyond 180)."""
    c = 0.5 * (lo + hi)
    return c + wrap180(np.asarray(a) - c)


def into_ranges(q, lim):
    """Posture (4 angles) written inside the servo ranges (see into_range)."""
    return tuple(float(into_range(q[i], lim[i][0], lim[i][1])) for i in range(4))


# ---------------------------------------------------------------- forward kinematics
def arm_points(g, q):
    """3D points of the arm for drawing. q = (base, shoulder, elbow, wrist) in degrees."""
    qb, t1, t2, t3 = (v * DEG for v in q)
    phi = t1 + t2 + t3
    cb, sb = math.cos(qb), math.sin(qb)

    def P(a, h, c):
        return np.array([a * cb - c * sb, h, a * sb + c * cb])

    cs, ce = g.lat_shoulder, g.lat_shoulder + g.lat_elbow
    aE, hE = g.R0 + g.L1 * math.cos(t1), g.H0 + g.L1 * math.sin(t1)
    aW, hW = aE + g.L2 * math.cos(t1 + t2), hE + g.L2 * math.sin(t1 + t2)
    aD, hD = aW - g.HP * math.sin(phi), hW + g.HP * math.cos(phi)
    aT, hT = aD + g.L3 * math.cos(phi), hD + g.L3 * math.sin(phi)
    pts = {
        "ground": P(0, 0, 0), "shoulder": P(g.R0, g.H0, 0), "shoulder2": P(g.R0, g.H0, cs),
        "elbow": P(aE, hE, cs), "elbow2": P(aE, hE, ce), "wrist": P(aW, hW, ce),
        "wrist2": P(aW, hW, g.C), "offset": P(aD, hD, g.C), "tip": P(aT, hT, g.C),
    }
    frame = {   # gripper axes in 3D: approach, perpendicular (in the arm plane), side
        "approach": np.array([math.cos(phi) * cb, math.sin(phi), math.cos(phi) * sb]),
        "perp": np.array([-math.sin(phi) * cb, math.cos(phi), -math.sin(phi) * sb]),
        "side": np.array([-sb, 0.0, cb]),
    }
    return pts, frame, phi / DEG


def fk(g, q):
    """Fingertip position (X, Y, Z) and gripper pitch phi (degrees)."""
    pts, _, phi = arm_points(g, q)
    return pts["tip"], phi


# ---------------------------------------------------------------- vectorised planar arm
def _planar_ik(g, xb, yb, phi, branch):
    """Planar 3-link arm. xb, yb: target seen from the shoulder; phi in radians.
    Returns t1, t2, t3 (radians) and the mask of solutions that exist (law of cosines)."""
    c, s = np.cos(phi), np.sin(phi)
    xw = xb - g.L3 * c + g.HP * s          # step back along the gripper...
    yw = yb - g.L3 * s - g.HP * c          # ...then along the perpendicular offset
    D = (g.L1 ** 2 + g.L2 ** 2 - xw * xw - yw * yw) / (2.0 * g.L1 * g.L2)
    exists = np.abs(D) <= 1.0
    alpha = np.arccos(np.clip(D, -1.0, 1.0))
    t2 = branch * (np.pi - alpha)
    t1 = np.arctan2(yw, xw) - np.arctan2(g.L2 * np.sin(t2), g.L1 + g.L2 * np.cos(t2))
    t3 = phi - t1 - t2
    return t1, t2, t3, exists


def _check_planar(g, lim, t1, t2, t3):
    """For planar angles (radians): margin to the limits (degrees, shoulder/elbow/wrist),
    and the masks 'within limits', 'clear of the table', 'clear of the base'."""
    q2 = into_range(t1 / DEG, *lim[1])
    q3 = into_range(t2 / DEG, *lim[2])
    q4 = into_range(t3 / DEG, *lim[3])
    margin = np.minimum.reduce([q2 - lim[1][0], lim[1][1] - q2, q3 - lim[2][0], lim[2][1] - q3,
                                q4 - lim[3][0], lim[3][1] - q4])
    phi = t1 + t2 + t3
    ca, sa = np.cos(phi), np.sin(phi)
    aE, hE = g.R0 + g.L1 * np.cos(t1), g.H0 + g.L1 * np.sin(t1)
    aW, hW = aE + g.L2 * np.cos(t1 + t2), hE + g.L2 * np.sin(t1 + t2)
    mt = g.table_margin
    table = (hE - g.servo_radius >= mt) & (hW - g.servo_radius >= mt)
    cs, ce, C = g.lat_shoulder, g.lat_shoulder + g.lat_elbow, g.C
    r2 = g.base_radius ** 2
    base = ((hE >= g.base_height) | (aE * aE + cs * cs >= r2)) & \
           ((hW >= g.base_height) | (aW * aW + ce * ce >= r2))
    corners = [(g.gripper_len_min, g.gripper_perp_min), (g.gripper_len_min, g.gripper_perp_max),
               (g.L3, g.gripper_perp_min), (g.L3, g.gripper_perp_max), (g.L3, g.HP)]
    for l, p in corners:   # gripper outline and fingertips
        a = aW + l * ca - p * sa
        h = hW + l * sa + p * ca
        table &= h >= mt
        base &= (h >= g.base_height) | (a * a + C * C >= r2)
    return margin, margin >= 0, table, base


# ---------------------------------------------------------------- inverse kinematics
@dataclass
class Solution:
    ok: bool
    q: tuple = None          # (base, shoulder, elbow, wrist) in degrees
    phi: float = None        # gripper pitch (degrees)
    side: int = 1            # +1: arm turned towards the target; -1: over the shoulder
    branch: int = -1         # sign of the elbow angle
    margin: float = 0.0      # degrees before the nearest limit (shoulder, elbow, wrist)
    reason: str = ""


def _branch_side(g, q):
    """Elbow branch and side (in front / over the shoulder) of a posture."""
    t1, t2, t3 = q[1] * DEG, q[2] * DEG, q[3] * DEG
    phi = t1 + t2 + t3
    a = g.R0 + g.L1 * math.cos(t1) + g.L2 * math.cos(t1 + t2) + g.L3 * math.cos(phi) - g.HP * math.sin(phi)
    return (1 if float(wrap180(q[2])) > 0 else -1), (1 if a >= 0 else -1)


def solve(g, lim, X, Y, Z, q_ref=None, phi_fixed=None, follow=False, phi_step=2.0, follow_window=40.0):
    """Improved inverse kinematics.

    For each side (towards the target / over the shoulder) and each elbow solution, every gripper pitch is tried
    (or only phi_fixed). Solutions outside the limits or colliding (table, base) are discarded, and the one with
    the lowest cost is kept:
      - follow=False (go to a position): far from the limits, gripper close to horizontal;
      - follow=True (follow the mouse): the closest to the current posture q_ref, without changing elbow solution
        or side while possible -> continuous motion, no jump. The search starts around the current gripper
        pitch (+-follow_window), which is faster.
    The chosen gripper pitch is then refined to a hundredth of a degree, it is not limited to phi_step.
    Returned angles are written inside the servo ranges (see into_range).
    """
    lim = np.asarray(lim, dtype=float)
    rho = math.hypot(X, Z)
    if rho <= abs(g.C) + 1e-6:
        return Solution(False, reason="too close to the base axis")
    yb = Y - g.H0
    ref_branch, ref_side, phi_ref = None, None, None
    if q_ref is not None:
        q_ref = into_ranges(q_ref, lim)
        ref_branch, ref_side = _branch_side(g, q_ref)
        phi_ref = float(wrap180(q_ref[1] + q_ref[2] + q_ref[3]))
    phi_pref = phi_ref if (follow and phi_ref is not None) else 0.0
    weights = np.array([0.6, 1.0, 1.0, 0.6])

    def evaluate(side, branch, xb, qb, ph):
        t1, t2, t3, exists = _planar_ik(g, xb, yb, ph, branch)
        margin, in_limits, table, base = _check_planar(g, lim, t1, t2, t3)
        valid = exists & in_limits & table & base
        q = np.stack([np.full_like(t1, qb), into_range(t1 / DEG, *lim[1]),
                      into_range(t2 / DEG, *lim[2]), into_range(t3 / DEG, *lim[3])])
        cost = (0.5 if follow else 4.0) * np.maximum(0.0, 20.0 - margin) ** 2
        cost = cost + (0.05 if follow else 0.5) * wrap180(ph / DEG - phi_pref) ** 2
        if q_ref is not None:
            d = q - np.asarray(q_ref, dtype=float)[:, None]   # actual travel of each servo
            cost = cost + (1.0 if follow else 0.03) * (weights[:, None] * d * d).sum(axis=0)
            if follow:
                cost = cost + 5000.0 * (branch != ref_branch) + 20000.0 * (side != ref_side)
        if side < 0:
            cost = cost + 2000.0
        cost = np.where(valid, cost, np.inf)
        return cost, q, exists, in_limits, table, base, margin

    def search(phis):
        best = None
        tally = {"base": 0, "reach": 0, "limits": 0, "table": 0, "base collision": 0, "tries": 0}
        for side in (1, -1):
            a = side * math.sqrt(rho * rho - g.C * g.C)
            qb = float(into_range((math.atan2(Z, X) - math.atan2(g.C, a)) / DEG, *lim[0]))
            if not (lim[0][0] <= qb <= lim[0][1]):
                tally["base"] += 1
                continue
            xb = a - g.R0
            for branch in (-1, 1):
                cost, q, exists, in_limits, table, base, margin = evaluate(side, branch, xb, qb, phis)
                tally["tries"] += 1
                tally["reach"] += int(exists.any())
                tally["limits"] += int((exists & in_limits).any())
                tally["table"] += int((exists & in_limits & table).any())
                tally["base collision"] += int((exists & in_limits & table & base).any())
                i = int(np.argmin(cost))
                if np.isfinite(cost[i]) and (best is None or cost[i] < best[0]):
                    best = (float(cost[i]), side, branch, xb, qb, float(phis[i]))
        return best, tally

    all_phis = np.arange(-180.0, 180.0, phi_step) * DEG
    if phi_fixed is not None:
        best, tally = search(np.array([phi_fixed * DEG]))
    elif follow and phi_ref is not None:
        best, tally = search((phi_ref + np.arange(-follow_window, follow_window + 1e-9, phi_step)) * DEG)
        if best is None:            # nothing near the current pitch: search everywhere
            best, tally = search(all_phis)
    else:
        best, tally = search(all_phis)
    if best is None:
        if tally["tries"] == 0:
            reason = "base angle outside its limits"
        elif tally["reach"] == 0:
            reason = "out of reach (too far, or too close to the shoulder)"
        elif tally["limits"] == 0:
            reason = "blocked by the servo limits"
        elif tally["table"] == 0:
            reason = "the arm would hit the table"
        else:
            reason = "the arm would hit the base"
        return Solution(False, reason=reason)

    _, side, branch, xb, qb, ph = best
    if phi_fixed is None:   # refine the gripper pitch: fine grids around the best try
        for width in (phi_step, phi_step / 20.0):
            tries = ph + np.linspace(-width, width, 41) * DEG
            cost = evaluate(side, branch, xb, qb, tries)[0]
            i = int(np.argmin(cost))
            if np.isfinite(cost[i]):
                ph = float(tries[i])
    cost, q, *_, margin = evaluate(side, branch, xb, qb, np.array([ph]))
    return Solution(True, q=tuple(float(v) for v in q[:, 0]), phi=float(wrap180(ph / DEG)), side=side,
                    branch=branch, margin=float(margin[0]))


def follow_path(g, lim, q0, target, phi_fixed=None, step_mm=8.0, max_jump=25.0, n_max=40):
    """Mouse following: go from the current position (posture q0) to the target in a straight line, in small steps,
    keeping a continuous posture (solve in follow mode at each step).
    If a step needs a sudden change of posture (more than max_jump degrees on one joint, for example going over
    the shoulder because the base reached its limit), the arm stops before it.
    Returns (postures, reason): postures = the intermediate postures (the last one is the furthest point reached),
    reason = "" if the target is reached, otherwise why the arm stopped."""
    lim = np.asarray(lim, dtype=float)
    q = into_ranges(q0, lim)
    p0 = fk(g, q)[0]
    target = np.asarray(target, dtype=float)
    n = int(min(n_max, max(1, math.ceil(float(np.linalg.norm(target - p0)) / step_mm))))
    postures = []
    for k in range(1, n + 1):
        p = p0 + (target - p0) * (k / n)
        s = solve(g, lim, *p, q_ref=q, phi_fixed=phi_fixed, follow=True)
        if not s.ok:
            return postures, s.reason
        jump = float(np.max(np.abs(np.subtract(s.q, q))))
        if jump > max_jump:
            return postures, "this would need a sudden change of posture (%.0f°): right-click to go there" % jump
        q = s.q
        postures.append(q)
    return postures, ""


def path_is_safe(g, lim, q0, q1, step_deg=2.0):
    """Is the motion from q0 to q1 done by the firmware (all servos together, in a straight line in joint space)
    safe? The end posture must respect the limits without touching the table or the base, and the motion must
    never ENTER a forbidden zone. It may LEAVE one: if the arm starts from a forbidden posture (for example outside
    the limits, left by an older program), that is the only way out.
    Returns (ok, reason)."""
    lim = np.asarray(lim, dtype=float)
    q0, q1 = np.array(into_ranges(q0, lim)), np.array(into_ranges(q1, lim))
    n = int(np.max(np.abs(q1 - q0)) / step_deg) + 2
    Q = q0 + np.linspace(0.0, 1.0, n)[:, None] * (q1 - q0)
    base_ok = (Q[:, 0] >= lim[0][0] - 1e-6) & (Q[:, 0] <= lim[0][1] + 1e-6)
    _, in_limits, table, base = _check_planar(g, lim, Q[:, 1] * DEG, Q[:, 2] * DEG, Q[:, 3] * DEG)
    for ok, reason in ((base_ok & in_limits, "outside the limits"), (table, "the arm would hit the table"),
                       (base, "the arm would hit the base")):
        if not ok[-1]:
            return False, reason + " at the end of the move"
        if not ok[int(np.argmax(ok)):].all():    # once out of the forbidden zone, never go back in
            return False, reason + " during the move"
    return True, ""


def posture_is_valid(g, lim, q):
    """Does posture q respect the limits, the table and the base? Returns (ok, reason, margin)."""
    lim = np.asarray(lim, dtype=float)
    q = into_ranges(q, lim)
    if not (lim[0][0] - 1e-6 <= q[0] <= lim[0][1] + 1e-6):
        return False, "base outside its limits", -1.0
    t = [np.array([q[i] * DEG]) for i in (1, 2, 3)]
    margin, in_limits, table, base = _check_planar(g, lim, *t)
    if not in_limits[0]:
        return False, "outside the limits", float(margin[0])
    if not table[0]:
        return False, "the arm would hit the table", float(margin[0])
    if not base[0]:
        return False, "the arm would hit the base", float(margin[0])
    return True, "", float(margin[0])


# ---------------------------------------------------------------- map of reachable positions
class ReachMap:
    """Map of the reachable positions, computed once in the arm plane (reach a, height y), then read for any view
    (top or side). phi_fixed: None = free gripper pitch; otherwise only this pitch is allowed."""

    def __init__(self, g, lim, phi_fixed=None, step=4.0, phi_step=3.0):
        self.g, self.lim, self.step = g, np.asarray(lim, dtype=float), step
        R = g.reach + step
        self.a = np.arange(-R, R + step / 2, step)
        self.y = np.arange(0.0, g.H0 + R + step / 2, step)
        phis = (np.array([phi_fixed]) if phi_fixed is not None else np.arange(-180, 180, phi_step)) * DEG
        self.ok = np.zeros((len(self.y), len(self.a)), dtype=bool)
        self.margin = np.full((len(self.y), len(self.a)), -1.0)
        xb = (self.a - g.R0)[:, None]
        ph = phis[None, :]
        for i, y in enumerate(self.y):
            for branch in (-1, 1):
                t1, t2, t3, exists = _planar_ik(g, xb, y - g.H0, ph, branch)
                margin, in_limits, table, base = _check_planar(g, self.lim, t1, t2, t3)
                valid = exists & in_limits & table & base
                m = np.where(valid, margin, -1.0).max(axis=1)
                self.ok[i] |= valid.any(axis=1)
                self.margin[i] = np.maximum(self.margin[i], m)

    def plane(self, a, y):
        """Reads the map of the arm plane (nearest cell)."""
        j = np.clip(np.rint((np.asarray(a) - self.a[0]) / self.step).astype(int), 0, len(self.a) - 1)
        i = np.clip(np.rint(np.asarray(y) / self.step).astype(int), 0, len(self.y) - 1)
        inside = (np.asarray(y) >= 0) & (np.asarray(y) <= self.y[-1])
        return self.ok[i, j] & inside, np.where(inside, self.margin[i, j], -1.0)

    def point(self, X, Y, Z):
        """Reachable? and margin, for 3D points (arrays of the same shape)."""
        X, Y, Z = np.broadcast_arrays(np.asarray(X, float), np.asarray(Y, float), np.asarray(Z, float))
        rho2 = X * X + Z * Z
        C = self.g.C
        far = rho2 > C * C
        ok = np.zeros(X.shape, dtype=bool)
        margin = np.full(X.shape, -1.0)
        theta = np.arctan2(Z, X)
        for side in (1, -1):
            a = side * np.sqrt(np.maximum(rho2 - C * C, 0.0))
            qb = into_range((theta - np.arctan2(C, a)) / DEG, *self.lim[0])
            base_ok = (qb >= self.lim[0][0]) & (qb <= self.lim[0][1]) & far
            o, m = self.plane(a, Y)
            o &= base_ok
            ok |= o
            margin = np.maximum(margin, np.where(o, m, -1.0))
        return ok, margin

    def nearest(self, X, Y, Z):
        """Nearest reachable point in the same vertical plane (same direction seen from above).
        Returns (X, Y, Z), or None if nothing is reachable in this direction."""
        theta = math.atan2(Z, X)
        rho = np.arange(0.0, self.g.reach + self.step, self.step / 2)
        R, H = np.meshgrid(rho, self.y)
        ok, _ = self.point(R * math.cos(theta), H, R * math.sin(theta))
        if not ok.any():
            return None
        d = (R - math.hypot(X, Z)) ** 2 + (H - Y) ** 2
        d[~ok] = np.inf
        i = np.unravel_index(np.argmin(d), d.shape)
        return float(R[i] * math.cos(theta)), float(H[i]), float(R[i] * math.sin(theta))
