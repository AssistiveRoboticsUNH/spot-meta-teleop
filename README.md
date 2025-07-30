# Spot-Meta-Teleop  
**Meta Quest 3 ✕ Boston Dynamics Spot SDK 5.0**  
Remote-operate Spot’s base, arm and gripper with natural 6-DoF hand motions.

---

## ✨  Key Features
| Capability | Details |
|------------|---------|
| **Full-body tele-operation** | Drive base with left-right joystick, servo arm & claw with right controller. |
| **Anchor-and-delta control** | Hold grip → anchor pose; move hand → end-effector follows, 30 Hz. |
| **Safe start-up / shutdown** | Script auto-acquires lease, clears Keepalive & Estop, undocks, stands, and power-offs cleanly. |
| **No ROS, no Unity** | Pure Python 3.10 on top of official `bosdyn-client == 5.0.0`. |
| **Quest 3 tracking pipeline** | Uses [`OculusReader`](https://github.com/rail-berkeley/oculus_reader) for sub-10 ms pose streaming. |

---

## Controller Key Map

ToDo

---

## 🛠  Prerequisites

| Component | Tested version |
|-----------|----------------|
| Python | **3.10** (conda recommended) |
| Spot SDK wheels | `bosdyn-client==5.0.0` `bosdyn-mission==5.0.0` `bosdyn-choreography-client==5.0.0` `bosdyn-orbit==5.0.0` |
| Meta Quest 3 | v66 firmware, **Developer Mode ON** |
| Network | Robot ↔ PC ↔ Quest on same low-latency subnet (≤ 2 ms) |

> **Safety** – Keep the OEM E-Stop within reach. Never run tele-op unattended.

---

## 📦 Installation

```bash
# 1. conda env
conda create -n spot python=3.10 -y
conda activate spot

# 2. clone
git clone https://github.com/mnakash/spot-meta-teleop.git
cd spot-meta-teleop

# 3. Install requirements
python -m pip install -r requirements.txt
```

## Setup Environment Variables
Open your .bashrc file in a text editor by `nano ~/.bashrc`.
Scroll to the bottom and add the following lines:
```
# Spot VR Teleop environment variables
export SPOT_ROBOT_IP=<robot_ip>
export BOSDYN_CLIENT_USERNAME=<username>
export BOSDYN_CLIENT_PASSWORD=<password>
```
Save and exit (Ctrl + O, Enter, then Ctrl + X in nano)

Apply changes immediately:
`source ~/.bashrc`

Verify they are set:
```
echo $SPOT_ROBOT_IP
echo $BOSDYN_CLIENT_USERNAME
echo $BOSDYN_CLIENT_PASSWORD
```


## Setup Meta controller
Follow guidelines here in [`OculusReader`](https://github.com/rail-berkeley/oculus_reader) repo for setting up Meta Quest Controller over WiFi or USB.

## How to run
1. Make sure to be able to run [reader.py](reader.py) before controlling spot.
2. Run `python3 teleop_spot.py` to start controlling spot.


## .npz file structure

Each saved .npz contains one “session” dictionary with the following keys:


| Key                   | Shape                 | Description                                                                        |
| --------------------- | --------------------- | ---------------------------------------------------------------------------------- |
| **images**            | `(N,)` (dtype=object) | A length‑N array of OpenCV BGR frames; each element is an `HxW×3` `uint8` ndarray. |
| **arm\_joint\_names** | `(J,)` (dtype=`<U…`)  | The J joint‐names (strings) for all recorded “arm0.\*” joints.                     |
| **arm\_q**            | `(N, J)` (float32)    | Joint positions at each timestep.                                                  |
| **arm\_dq**           | `(N, J)` (float32)    | Joint velocities at each timestep.                                                 |
| **ee\_pose**          | `(N, 7)` (float32)    | End‑effector pose in body frame: `[tx,ty,tz,qx,qy,qz,qw]`.                         |
| **vision\_in\_body**  | `(N, 7)` (float32)    | Vision‑frame origin expressed in body frame (same format as `ee_pose`).            |
| **body\_vel**         | `(N, 6)` (float32)    | Body linear & angular velocity in vision frame: `[vx,vy,vz,wx,wy,wz]`.             |
| **gripper**           | `(N, 1)` (float32)    | Gripper opening percentage.                                                        |
| **ee\_force**         | `(N, 3)` (float32)    | Estimated end‑effector force vector in hand.                                       |
| **t**                 | `(N, 1)` (float64)    | Timestamp for each capture, in seconds (including fractional).                     |

## 📂 Dataset Structure (.h5)
The dataset is organized in the following hierarchy:

```text
arm_joint_names                (shape=(7,), dtype=|S8)
demo_0/
├── actions                    (shape=(N, 10), dtype=float32)
└── obs/
    ├── arm_dq                 (shape=(N, 7), dtype=float32)
    ├── arm_q                  (shape=(N, 7), dtype=float32)
    ├── body_vel               (shape=(N, 6), dtype=float32)
    ├── ee_force               (shape=(N, 3), dtype=float32)
    ├── eef_pos                (shape=(N, 3), dtype=float32)
    ├── eef_quat               (shape=(N, 4), dtype=float32)
    ├── gripper                (shape=(N, 1), dtype=float32)
    ├── images_0               (shape=(N, 480, 640, 3), dtype=uint8)
    ├── t                      (shape=(N, 1), dtype=float32)
    └── vision_in_body         (shape=(N, 7), dtype=float32)
demo_1/
...
demo_N/