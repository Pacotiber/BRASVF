# Teleoperation with stereo cameras – open-source arms with MG995 servos

Goal: **control the arm by copying the motion of a human arm**, tracked by **two cameras (stereo)**.

This document lists the open-source arms compatible with the MG995 / MG996R that can be downloaded, the software building blocks that can be reused for teleoperation, and why each one is interesting.

> Research done on 13/10/2026. The Thingiverse, MyMiniFactory, Cults3D and MakerWorld pages block automated reading: their information comes from search summaries and is marked **(to be checked)**. The GitHub repositories were read directly.

---

## 1. Target teleoperation chain

```
 2 cameras ──► MediaPipe Pose (2D, on each image) ──► stereo triangulation ──► 3D points shoulder / elbow / wrist / hand
                                                                                         │
 MG995 servos ◄── PCA9685 ◄── Arduino / ESP32 / Raspberry Pi ◄── smoothing + limits ◄── robot joint angles
```

1. **Stereo calibration** of the 2 cameras (checkerboard, OpenCV).
2. **2D detection** of the joints on each image (MediaPipe Pose, plus MediaPipe Hands for the gripper).
3. **Triangulation** of the matching points: this gives 3D positions in mm, much more reliable than the depth "guessed" by a single camera.
4. **Angle computation**: shoulder yaw and pitch (arm vector), elbow (arm/forearm angle), wrist pitch, gripper opening (thumb-index distance).
5. **Copy onto the robot**: joint limits, smoothing, speed limit, then the commands are sent to the servos.

---

## 2. Printable arms compatible with the MG995 / MG996R

The MG996R is the improved version of the MG995: same size, same mounting, same 25T spline. Parts designed for one fit the other.

### ⭐ 2.1 6-axis "Arduino MG996r arm" – Emre Kalem (MakerWorld) – **free**
- **Link:** https://makerworld.com/en/models/1055189-arduino-mg996r-arm
- **Servos:** 4 × MG995 or MG996R, 3 × MG90S *(to be checked)*.
- **Bearings:** one 608 and two 6203 *(to be checked)*.
- **Provided:** source code and wiring diagrams linked from the page. Designed for FDM printing, with very few supports.
- **Why use it:**
  - **6-axis anthropomorphic architecture** (base, shoulder, elbow, wrist). The easiest to map onto the human arm: each human joint has its equivalent.
  - **Designed for MG995**, so your servos can be reused.
  - **Bearings on the load-bearing joints**: the weight is carried by the bearings and not by the servo shafts, which fixes exactly the weakness of the base of our current arm.
- **Limit:** STEP files not confirmed (maybe STL only).

### 2.2 OmArm Zero – OmArTronics (Cults3D)
- **Link:** https://cults3d.com/en/3d-model/gadget/omarm-zero-6-dof-5-1-gripper-robotic-arm-esp32-wifi-control-stl-step
- **Servos:** 3 × MG996R (base, shoulder, elbow) and 3 × SG90 (wrist, gripper).
- **Electronics:** ESP32 + PCA9685.
- **Provided:** STL + STEP + firmware + PDF guide, tutorial, wiring diagram, video.
- **Licence:** personal, non-commercial use. Check the price on the page.
- **Why use it:**
  - **The most complete documentation** of the selection, with the **STEP files** (editable in SolidWorks).
  - **USB serial link designed for ROS 2** and built-in Wi-Fi control: you only need to send the angles computed by the vision.
- **Limit:** SG90 at the wrist, so little torque at the wrist.

### 2.3 EEZYbotARM MK2 – Carlo Franciscone (MyMiniFactory / Onshape)
- **Links:**
  - files: https://www.myminifactory.com/object/3d-print-eezybotarm-mk2-46620
  - source CAD: on the author's Onshape account, linked from the page
  - ROS software: https://github.com/contagon/eezy_arm_ros
- **Servos:** 3 standard-size servos (MG995 / MG946R depending on the build *(to be checked)*) + 1 SG90 for the gripper.
- **Architecture:** 3 axes + gripper, joints on M4 screws with lock nuts.
- **Licence:** free, with attribution requested by the author.
- **Why use it:**
  - **Parallelogram**: the 3 servos stay **in the base**, so the arm is light and fast. Ideal to follow a human motion without jerks.
  - **Very widespread project**, with a large community and many remixes.
  - **Existing ROS package** with inverse kinematics (`eezy_arm_ros`).
- **Limit:** no orientable wrist. It copies the **position of the hand** (x, y, z), not the orientation.

### 2.4 3D-printed-Robotic-Arm – Abhi28-eng (GitHub)
- **Link:** https://github.com/Abhi28-eng/3D-printed-Robotic-Arm
- **Servos:** 7 servos (MG996R + MG90S).
- **Electronics:** Raspberry Pi Pico + PCA9685.
- **Provided:** dozens of STL files in the repository (gears, grippers, servo mounts, electronics box) and a PDF wiring guide.
- **Software:** MicroPython firmware with **calibration, interpolation, safety and real-time teleoperation**, currently over radio (ESP32 + NRF24L01).
- **Why use it:**
  - The firmware already handles what is needed on the robot side (interpolation, safety). Only the radio controller has to be replaced by vision.
  - The STL files can be downloaded directly.
- **Limit:** no licence stated. Ask the author before distributing a modified version.

### 2.5 Robot-Arm – greendino2000 (GitHub, MIT)
- **Link:** https://github.com/greendino2000/Robot-Arm
- **Architecture:** 4 axes, with 1 × MG996R, 1 × SG90 and 1 NEMA 17 for the base.
- **Provided:** STL files in the `Model STLs` folder.
- **Software:** simple serial commands (`A180` = base at 180°), plus a web interface.
- **Why use it:**
  - **MIT licence**, the most permissive of the selection.
  - Very simple serial protocol to drive from Python.
  - Stepper-motor base, so no servo carries the weight.
- **Limit:** assembly guide still being written.

### Design reference (no MG995): OmArm One
https://github.com/omartronics/OmArm-One. It uses DS3240 servos and its CAD is paid. Worth a look for one principle: **every joint that carries a moment turns on a bearing** (51110 thrust bearing at the base, 695ZZ on the pitch joints). The repository (MIT licence) also contains an analysis script: kinematics, statics, payload, DH parameters.

---

## 3. Reusable teleoperation software

| Project | Link | What it brings |
|---|---|---|
| **bodypose3d** (TemugeB, MIT) | https://github.com/TemugeB/bodypose3d | **The core of stereo tracking**: 2 cameras, MediaPipe in 2D, **triangulation** → 3D body points **in real time**. The repository also points to its stereo calibration guide. |
| **joint_angles_calculate** (TemugeB) | https://github.com/TemugeB/joint_angles_calculate | Computes the **joint angles** from a 3D pose (a frame at each joint, T-pose reference). |
| **gesture-controlled-robotic-arm** (Rayner305) | https://github.com/Rayner305/gesture-controlled-robotic-arm | Complete chain with **6 × MG996R**: MediaPipe on a PC, **UDP/JSON** to a Raspberry Pi, **PCA9685**, **per-servo smoothing** and **per-servo limits**. The best model for the "robot" side. No licence stated. |
| **Vision-Guided-3DOF-Arm** (akshitnarwal03) | https://github.com/akshitnarwal03/Vision-Guided-3DOF-Arm | MG995 + Arduino + MediaPipe, with a **calibration tool** and tests. Warns against stalling an MG995 against a limit. |
| **6DOF-robotic-arm** (dreamerarun) | https://github.com/dreamerarun/6DOF-robotic-arm | MG995 + SG90 + PCA9685, web interface and a **ROS 2 Humble / MoveIt 2 bridge** with a URDF (lengths to replace with the real ones). |
| **arm_eval** (rus1ru, MIT) | https://github.com/rus1ru/arm_eval | **MuJoCo simulator** of a printed 6-axis arm with **MG995** (torque limited to 1.13 N·m): test the teleoperation and the **servo sag** before the hardware. |
| **naoqi-pose-retargeting** (elggem) | https://github.com/elggem/naoqi-pose-retargeting | Example of **retargeting** a MediaPipe pose onto a robot (Pepper / Nao). |

---

## 4. Recommendation

**For an arm that imitates the human arm:**
1. **Mechanics:** the **6-axis arm by Emre Kalem (§2.1)**. It is anthropomorphic, designed for MG995 and mounted on bearings. If you want STEP files to modify, take **OmArm Zero (§2.2)**.
2. **Vision:** **bodypose3d** (stereo triangulation), then **joint_angles_calculate** for the angles.
3. **Robot side:** reuse the logic of **Rayner305** (PCA9685, smoothing, limits, UDP) on a Raspberry Pi or an ESP32.
4. **Tuning:** start in the **arm_eval** simulator, then move to the real arm.

**Other option:** the same software chain also works with **your current arm** (4 × MG995 + SG90). You only need to write the mapping between the human angles and its joints.

---

## 5. Points of attention

- **Power supply:** an MG995/MG996R can draw **2.5 A** when it stalls. Six servos starting together under load can exceed 10 A. Plan a **5–6 V, 10 A power supply**, separate from the control board, with a common ground.
- **Accuracy:** the MG995 has a fairly wide dead band and average centring. Add **smoothing** (low-pass filter, or a gradual approach to the command as Rayner305 does) and a **speed limit**, otherwise the arm shakes.
- **Wrist rotation:** vision alone poorly estimates rotations about the axis of a limb (wrist rotation, elbow yaw), especially when one limb hides another. Drive the wrist rotation separately (gesture, button) or keep it fixed.
- **Occlusions:** force a **constant length** for the arm and the forearm after triangulation. This is the method of the arXiv paper 2606.19240 and it removes many jumps.
- **Safety:** software limits on every joint, **stop if the vision loses the arm** (timeout, like Rayner305's safe server), and an emergency stop button on the servo power supply.
- **Licences:** check every licence before distributing a modified version. Abhi28's and Rayner305's are not stated, and OmArm's is restricted to non-commercial use.

---

## Sources
- [Arduino MG996r arm – MakerWorld](https://makerworld.com/en/models/1055189-arduino-mg996r-arm)
- [OmArm Zero – Cults3D](https://cults3d.com/en/3d-model/gadget/omarm-zero-6-dof-5-1-gripper-robotic-arm-esp32-wifi-control-stl-step)
- [EEZYbotARM Mk2 – MyMiniFactory](https://www.myminifactory.com/object/3d-print-eezybotarm-mk2-46620)
- [eezy_arm_ros – GitHub](https://github.com/contagon/eezy_arm_ros)
- [EEZYbotARM – Core Electronics forum (MK2 servos)](https://forum.core-electronics.com.au/t/eezybotarm-arduino-control/4162)
- [Abhi28-eng/3D-printed-Robotic-Arm](https://github.com/Abhi28-eng/3D-printed-Robotic-Arm)
- [greendino2000/Robot-Arm](https://github.com/greendino2000/Robot-Arm)
- [omartronics/OmArm-One](https://github.com/omartronics/OmArm-One)
- [TemugeB/bodypose3d](https://github.com/TemugeB/bodypose3d)
- [TemugeB/joint_angles_calculate](https://github.com/TemugeB/joint_angles_calculate)
- [Rayner305/gesture-controlled-robotic-arm](https://github.com/Rayner305/gesture-controlled-robotic-arm)
- [akshitnarwal03/Vision-Guided-3DOF-Arm](https://github.com/akshitnarwal03/Vision-Guided-3DOF-Arm)
- [dreamerarun/6DOF-robotic-arm](https://github.com/dreamerarun/6DOF-robotic-arm)
- [rus1ru/arm_eval](https://github.com/rus1ru/arm_eval)
- [elggem/naoqi-pose-retargeting](https://github.com/elggem/naoqi-pose-retargeting)
- [Arm Kinematic Correction for Robot Teleoperation – arXiv 2606.19240](https://arxiv.org/pdf/2606.19240)
- [Teleoperation System Using a VR Headset and 3D Pose Estimation – Sensors](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12845624/)
- [Real Time Markerless 3D Hand Tracking for Intuitive Robotic Arm Control – Cal Poly](https://digitalcommons.calpoly.edu/theses/3200/)
- [MG996R / MG995 – data sheet](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2017/kmv48_rp625/kmv48_rp625/kmv48_rp625/MG996RTowerPro.pdf)
- [Servos for Robotic Arm – Arduino forum](https://forum.arduino.cc/t/servos-for-robotic-arm/368858)
