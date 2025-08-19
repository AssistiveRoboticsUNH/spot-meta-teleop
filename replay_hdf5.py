'''
This script is used to control spot from a provided h5 dataset file.
It uses the absolute positions to control the arm

Author: Moniruzzaman Akash
'''

import os
import h5py, time
import numpy as np
from spot_controller import SpotRobotController
from bosdyn.client.math_helpers  import SE3Pose, Quat

robot_ip = os.environ.get("SPOT_ROBOT_IP", "192.168.1.138")
user     = os.environ.get("BOSDYN_CLIENT_USERNAME", "user")
password = os.environ.get("BOSDYN_CLIENT_PASSWORD", "password")

print(f"Connecting to Spot at {robot_ip} ...")
print(f"user: {user}, password: {password}")

controller = SpotRobotController(robot_ip, user, password)
controller.undock()

controller.unstow_arm()

# Open the HDF5 file
try:
    hdf5_file_path = "demo_single.h5"
    with h5py.File(hdf5_file_path, 'r') as f:
        # Read the data
        demos = f['data']
        demo = demos['demo_0']
        positions = demo['obs']['eef_pos']
        orientations = demo['obs']['eef_quat']
        timestamps = demo['obs']['t']
        gripper_states = demo['obs']['gripper']

        for i in range(len(positions)):
            position = positions[i]
            orientation = orientations[i]
            timestamp = float(timestamps[i][0])
            gripper_state = float(gripper_states[i][0])

            # Send the command to the robot
            controller.move_arm_to(pos_xyz=position,
                                    quat_xyzw=orientation,
                                    gripper=gripper_state)  # Example gripper value

            # Print the current step
            print(f"Step {i+1}/{len(positions)}: Timestamp {timestamp}, Position {position}, Orientation {orientation}")

            # wait for timestamp amount of time
            if i < len(timestamps) - 1:
                next_timestamp = float(timestamps[i + 1][0])
                wait_time = next_timestamp - timestamp
                time.sleep(wait_time)
except Exception as e:
    print(f"An error occurred while reading the HDF5 file: {e}")

time.sleep(1) # wait for the robot to undock
controller.stow_arm()
controller.dock()