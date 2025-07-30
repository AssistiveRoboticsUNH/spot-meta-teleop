#!/usr/bin/env python3
"""
=================
Natural tele-operation of Boston Dynamics Spot and its arm with Meta Quest controllers.

The script assumes you already have:
    - Spot SDK 5.0.0 installed in the active conda env
    - A working meta quest setup with adb enabled
    - The `reader.py` module in the same directory, which reads the Meta Quest controller

Author: Moniruzzaman Akash
"""
import argparse, math, signal, sys, time, threading, os
import numpy as np
from typing import Tuple, Dict

from bosdyn.api import geometry_pb2
from bosdyn.client import create_standard_sdk, robot_command
from bosdyn.client.estop   import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease   import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
from bosdyn.client.power   import PowerClient
from bosdyn.client.docking import DockingClient, blocking_undock, blocking_dock_robot, get_dock_id
from bosdyn.client.robot_command import RobotCommandBuilder, blocking_stand, blocking_sit, block_until_arm_arrives
from bosdyn.client.gripper_camera_param import GripperCameraParamClient
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_state   import RobotStateClient
from bosdyn.client.math_helpers  import SE3Pose, Quat
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME

from reader import OculusReader
from demo_recorder import DemoRecorder
from utils.spot_utils import mat_to_se3, reaxis, rot_to_quat, proto_to_cv2, image_to_cv
from spot_images import SpotImages
import logging, cv2


class SpotVRTeleop:
    MAX_VEL_X = 0.6          # [m/s] forward/back
    MAX_VEL_Y = 0.6          # [m/s] left/right
    MAX_YAW   = 0.8          # [rad/s] spin
    ARM_SCALE = 2.0          # [m per m] controller-to-arm translation
    VELOCITY_CMD_DURATION = 0.2  # seconds

    def __init__(self, robot_ip, username, password, meta_quest_ip=None, demo_image_preview=True):
        self.oculus_reader = OculusReader(ip_address=meta_quest_ip)

        sdk = create_standard_sdk("spot-teleop")
        self.robot = sdk.create_robot(robot_ip)
        self.robot.authenticate(username, password)
        self.robot.time_sync.wait_for_sync()

        # ── LEASE ─────────────────────────────────────────
        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        try:
            self.lease = self.lease_client.acquire()
        except ResourceAlreadyClaimedError:
            self.lease = self.lease_client.take()
        self.robot.lease_wallet.add(self.lease)
        self.lease_keepalive = LeaseKeepAlive(self.lease_client)

        # ── E-STOP (simple) ───────────────────────────────
        estop_client = self.robot.ensure_client("estop")
        self.estop_endpoint = EstopEndpoint(estop_client, "vr_teleop", 9.0)
        self.estop_endpoint.force_simple_setup()
        self.estop_keepalive = EstopKeepAlive(self.estop_endpoint)
        self.estop_keepalive.allow()


        # command/state clients
        self.command_client   = self.robot.ensure_client("robot-command")
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)


        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.gripper_cam_param_client = self.robot.ensure_client(GripperCameraParamClient.default_service_name)
        self.logger = logging.getLogger()
        self.spot_images = SpotImages(
            self.robot,
            self.logger,
            self.image_client,
            self.gripper_cam_param_client
        )


        # ---- runtime vars ---
        self.arm_anchor_ctrl  = None   # 4×4 SE(3) when grip first pressed
        self.arm_anchor_robot = None   # SE3Pose   ^ … corresponding robot pose
        self.prev_r_grip      = False
        self.base_enabled     = False

        self.stowed = False  # arm stowed at start
        self.demo_image_preview = demo_image_preview  # show camera preview in demo mode1

        signal.signal(signal.SIGINT, self._clean_shutdown)


        # ── POWER ON ───────────────────────────────
        if not self.robot.is_powered_on():
            print("> Powering on...")
            self.robot.power_on(timeout_sec=20)
        print("> Robot is powered on.")

        self.dock_id = get_dock_id(self.robot)

        self.recorder = DemoRecorder(
            robot=self.robot,
            spot_images=self.spot_images,
            state_client=self.state_client,
            out_dir="demos",
            fps=10,
            preview=self.demo_image_preview)

    def get_meta_quest(self):
        return self.oculus_reader.get_transformations_and_buttons()

    def undock(self):
        try:
            self.dock_id = get_dock_id(self.robot)
            if self.dock_id is not None:
                print(f"Robot is docked at {self.dock_id} → undocking …")
                blocking_undock(self.robot, timeout=20)
                self.dock_id = get_dock_id(self.robot)
                print("Robot undocked.")
                return True
            else:
                print("Robot is not docked.")
                return False
        except Exception as e:
            print(f"[!] Error during undocking: {e}")
            return False
    def dock(self):
        try:
            self.dock_id = get_dock_id(self.robot)
            if self.dock_id is None:
                print("Robot is undocked → docking ...")
                # Stand before trying to dock.
                blocking_stand(self.command_client, timeout_sec=10)
                blocking_dock_robot(self.robot, dock_id=520, timeout=30)
                self.dock_id = get_dock_id(self.robot)
                print(f"Robot docked at id={self.dock_id} and powered off.")
                return True
            else:
                print(f"Robot is already docked at {self.dock_id}.")
                return False
        except Exception as e:
            print(f"[!] Error during docking: {e}")
            return False

    def dock_undock(self):
        try:
            self.dock_id = get_dock_id(self.robot)
            if self.dock_id is not None:
                print(f"Robot is docked at {self.dock_id} → undocking …")
                blocking_undock(self.robot, timeout=20)
                self.dock_id = get_dock_id(self.robot)
                print("Robot undocked.")
            else:
                print("Robot is undocked → docking ...")
                # Stand before trying to dock.
                blocking_stand(self.command_client, timeout_sec=10)
                blocking_dock_robot(self.robot, dock_id=520, timeout=30)
                self.dock_id = get_dock_id(self.robot)
                print(f"Robot docked at id={self.dock_id} and powered off.")
                print("> Powering on again...")
                self.robot.power_on(timeout_sec=20)
                print("> Robot is powered on.")
        except Exception as e:
            print(f"[!] Error during docking/undocking: {e}")
        
    def stand(self):
        # self.robot.power_on(timeout_sec=20.0)
        blocking_stand(self.command_client, timeout_sec=10)

    def sit(self):
        blocking_sit(self.command_client, timeout_sec=10)

    def move(self, vx: float, vy: float, wz: float):
        """Send a velocity command to the robot base."""
        try:
            vel_cmd = RobotCommandBuilder.synchro_velocity_command(vx, vy, wz)
            self.command_client.robot_command(vel_cmd, end_time_secs=time.time() + self.VELOCITY_CMD_DURATION)
        except Exception as e:
            print(f"[!] Error sending velocity command: {e}")

    def move_arm_to(self, pose):
        """Move the arm to a specified pose in vision frame."""
        # print goal pose
        print(f"Moving arm to: {pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}")

        try:
            # Convert SE3Pose to protobuf SE3Pose
            pose_pb = geometry_pb2.SE3Pose(
                position=geometry_pb2.Vec3(x=pose.x, y=pose.y, z=pose.z),
                rotation=geometry_pb2.Quaternion(w=pose.rot.w, x=pose.rot.x,
                                                 y=pose.rot.y, z=pose.rot.z)
            )
            arm_cmd = RobotCommandBuilder.arm_pose_command_from_pose(
                hand_pose=pose_pb,
                frame_name=VISION_FRAME_NAME,
                seconds=self.VELOCITY_CMD_DURATION
            )
            self.command_client.robot_command(arm_cmd, end_time_secs=time.time() + self.VELOCITY_CMD_DURATION)
        except Exception as e:
            print(f"[!] Error sending arm command: {e}")

    def stow_arm(self):
        """Stow the arm to a safe position."""
        print("Stowing arm...")
        try:
            stow_cmd = RobotCommandBuilder.arm_stow_command()
            cmd_id = self.command_client.robot_command(stow_cmd, end_time_secs=time.time() + 5.0)
            block_until_arm_arrives(self.command_client, cmd_id)
            self.stowed = True
            print("Arm stowed.")
        except Exception as e:
            print(f"[!] Error stowing arm: {e}")

    def unstow_arm(self):
        """Unstow the arm to a ready position."""
        print("Unstowing arm...")
        try:
            unstow_cmd = RobotCommandBuilder.arm_ready_command()
            cmd_id = self.command_client.robot_command(unstow_cmd, end_time_secs=time.time() + 5.0)
            block_until_arm_arrives(self.command_client, cmd_id)
            self.stowed = False
            print("Arm unstowed.")
        except Exception as e:
            print(f"[!] Error unstowing arm: {e}")

    def run(self):
        rate_hz = 30.0
        dt = 1.0 / rate_hz
        t_prev = time.time()
        
        while True:
            # --- Camera preview (runs every loop; throttle if needed) -------------
            # try:
            #     img_resp = self.spot_images.get_rgb_image("hand_color_image")   # pick any getter you like
            #     if img_resp is not None:
            #         frame = image_to_cv(img_resp)
            #         cv2.imshow("Hand", frame)
            #         cv2.waitKey(1)      # keeps the window responsive
            # except Exception as e:
            #     print(f"[!] Could not display image: {e}")

            # state = self.state_client.get_robot_state()  # keep state fresh
            # print(f"Robot state: {state}")


            poses, buttons = self.get_meta_quest()

            if len(poses) == 0 or len(buttons) == 0:
                print("[!] No poses or buttons received from Meta Quest.")
                time.sleep(dt)
                continue

            # ---------------- POWER TOGGLE (A/B) ------------------
            if buttons.get('A', False):
                # self.stand()
                self.dock_undock()
            if buttons.get('B', False):
                # self.sit()
                # self._clean_shutdown()
                self.recorder.start() if not self.recorder.is_recording else self.recorder.stop()

            # ---------------- BASE  (LEFT HAND) -------------------
            lgrip  = buttons.get('leftGrip', (0.0,))[0] > 0.5 or buttons.get('LG', False)
            ljx, ljy = buttons.get('leftJS', (0.0, 0.0))
            rjx, rjy = buttons.get('rightJS', (0.0, 0.0))

            try:
                vx  =  self.MAX_VEL_X * (ljy)        # up on joystick = +y in Quest
                vy  =  self.MAX_VEL_Y * (-ljx)         # right on joystick = +x in Quest
                wz  =  self.MAX_YAW   * (-rjx)         # right on JS -> negative yaw
                if self.dock_id is None: # If not docked, allow base movement
                    self.move(vx, vy, wz)

            except Exception as e:
                print(f"[!] Error sending base command: {e}")

            # ---------------- ARM   (RIGHT HAND) -------------------
            try:
                lgrip = buttons.get('leftGrip', (0.0,))[0] > 0.5 or buttons.get('LG', False)
                # rTrig = buttons.get('rightTrig', (0.0,))[0]
                rmat_raw  = poses['r']
                # Meta Quest arm frame has z-down, x-right coordinate system.
                # Reaxis to convert to robot hand frame: x-forward, y-left, z-up
                rmat = reaxis(rmat_raw)

                if lgrip and not self.prev_r_grip:
                    # first frame with grip pressed -> anchor
                    self.arm_anchor_ctrl  = rmat.copy()
                    self.arm_anchor_robot = self._current_ee_pose()
                if lgrip:
                    self.stowed = False  # arm is unstowed when moving arm
                    # Cartesian delta = anchor^{-1} * current
                    # Compute controller delta in anchor frame
                    delta = np.linalg.inv(self.arm_anchor_ctrl) @ rmat

                    delta_scaled = delta.copy()
                    delta_scaled[0:3, 3] *= self.ARM_SCALE  # scale translation
                    delta_pose = mat_to_se3(delta_scaled)

                    # Compose with robot-space anchor to get absolute target
                    goal = self.arm_anchor_robot * delta_pose  # composition

                    self.move_arm_to(goal)

                self.prev_r_grip = lgrip
            except Exception as e:
                print(f"[!] Error sending arm command: {e}")

            # ------------------ STOW/UNSTOW ARM -------------------
            if buttons.get('X', False):
                self.stow_arm()
            if buttons.get('Y', False):
                self.unstow_arm()

            # ---------------- GRIPPER TRIGGER ----------------------
            trigger_val = 1- buttons.get('rightTrig', (0.0,))[0] # use reverse of left trigger
            grip_cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(trigger_val)
            self.command_client.robot_command(grip_cmd)

            # ------------- keep teleop RT responsive --------------
            sleep = max(0.0, dt - (time.time() - t_prev))
            time.sleep(sleep)
            t_prev = time.time()

            if self.demo_image_preview:
                self.recorder.poll_preview()

    # ---------------------------------------------------------------------#
    #      HELPERS                                                         #
    # ---------------------------------------------------------------------#
    def _current_ee_pose(self) -> SE3Pose:
        """End-effector pose in vision frame."""
        state = self.state_client.get_robot_state()
        return get_a_tform_b(state.kinematic_state.transforms_snapshot,
                             VISION_FRAME_NAME, "hand")

    def _clean_shutdown(self, *_):
        print("\n[!] Shutting down VR teleop ...")
        try:
            self.sit()
            # self.oculus_reader.stop()
        finally:
            self.estop_keepalive._end_periodic_check_in()
            self.estop_keepalive.stop()
            sys.exit(0)

def main():
    robot_ip = os.environ.get("SPOT_ROBOT_IP", "192.168.1.138")
    user     = os.environ.get("BOSDYN_CLIENT_USERNAME", "user")
    password = os.environ.get("BOSDYN_CLIENT_PASSWORD", "password")

    print(f"Connecting to Spot at {robot_ip} ...")
    print(f"user: {user}, password: {password}")

    meta_ip = "192.168.1.37"
    teleop = SpotVRTeleop(robot_ip, user, password, meta_quest_ip= meta_ip, demo_image_preview=False)
    teleop.run()

if __name__ == "__main__":
    main()
