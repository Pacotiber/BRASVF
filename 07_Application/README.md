# Control application

| File | Role |
|---|---|
| `arm_app.py` | the application (run this one) |
| `kinematics.py` | arm model, inverse kinematics, map of reachable positions |
| `calibration.py` | servo calibration and the `arm_config.json` file |
| `robot_link.py` | serial link with the ESP32, and simulated robot |

```
pip install numpy pillow pyserial
python arm_app.py
```

The firmware to upload to the ESP32 is `../06_ESP32/arm_bridge/arm_bridge.ino`. Without `pyserial` or without a robot, the application works in "Simulation (no robot)" mode.

`arm_config.json` is created next to the application. It holds the calibration, the limits, the geometry, the rest position and the speed: keep it, it is your calibration. If an older `config_bras.json` (French version of the application) is found, it is converted automatically at the first start.

## Calibration, joint by joint

1. Select the joint (Calibration tab) and keep « Send to the servo live » ticked.
2. With +10 / −10, move the real arm to an easy-to-recognise position, then click the button that describes it (for example « 0°: arm 2 aligned with arm 1 »).
3. Do the same for a clearly different position (for example « −90°: arm 2 folded DOWN »). Two points fix the offset and the direction; with 3 points or more the mapping follows the servo exactly.
4. Record both limits (« Low limit = here », « High limit = here »).

The 3D view of the tab must move like the real arm. If it turns the other way, a point is wrong: delete it and do it again.

## Main checks done before each command

- the end posture respects the limits, the table and the base;
- the real path of the servos (straight line in joint space, from the robot's position and from the previous command) never enters a forbidden zone;
- each pulse width stays between the recorded limits of its servo.
