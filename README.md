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
| **Quest 3 tracking pipeline** | Uses [`OculusReader`](https://github.com/OculusQuestCode/OculusReader) for sub-10 ms pose streaming. |

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
Follow guidelines here in [`OculusReader`](https://github.com/OculusQuestCode/OculusReader) repo for setting up Meta Quest Controller over WiFi or USB.

## How to run
1. Make sure to be able to run [reader.py](reader.py) before controlling spot.
2. Run `python3 control_spot.py` to start controlling spot.