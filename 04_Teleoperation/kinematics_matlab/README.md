# Arm kinematic model (MATLAB)

Plain MATLAB, no toolbox needed. This is the first model of the arm (older dimensions: H0 = 107.6 mm, previous gripper). The current reference model is `07_Application/kinematics.py`.

| File | Role |
|---|---|
| `arm_params.m` | Dimensions measured in the CAD model (mm), sideways offsets, CAD pose, limits |
| `arm_fk.m` | Forward kinematics: returns the 4×4 homogeneous matrix of each frame |
| `arm_ik.m` | Analytical inverse kinematics (closed-form formulas), with an "up" or "down" elbow |
| `arm_plot.m` | 3D drawing of the arm with its frames (x red, y green, z blue) |
| `demo_arm.m` | Checks, then an interface with sliders and a target for the inverse kinematics |
| `my_arm_offsets.m` | 3D arm with the base and the sideways offsets (X forward, Y up, Z right) |
| `arm_3d_model.m` | Same model with the gripper, servo cylinders and a safe inverse kinematics (limits + floor) |

**Run:** open this folder in MATLAB, then type `demo_arm`.

## Frames
- **0 base**: on the base axis, on the floor. z up, x forward.
- **2 shoulder, 3 elbow, 4 wrist**: on the axis of each servo. x along the arm, **y = rotation axis** (pitch).
- The base rotates about **z**.

## Chain
```
T = Rz(q1) · Tr(r0, −32, z0) · Ry(−q2) · Tr(L1, 49.5, 0) · Ry(−q3) · Tr(L2, −7.25, 0) · Ry(−q4) [· Tr(L3, −30.62, 0)]
```

## Checked (MATLAB R2024b)
- CAD pose q = (0 ; 57.03 ; 8.62)° → wrist at (143.62 ; 10.25 ; 346.36) mm, same as SolidWorks.
- Inverse kinematics of this point → q = (0 ; 57.03 ; 8.62)° with the "down" elbow.
- Forward → inverse → forward round trip on 500 random poses: error < 1e-12 mm.
