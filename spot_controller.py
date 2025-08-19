#!/usr/bin/env python3
"""
spot_controller.py
-------------------
Utility class to command Boston Dynamics Spot's arm by either

1. apply_action(10-D vector)
     [ Δx Δy Δz,  rot6d(6),  gripper_abs ]

2. send_pose(position_xyz, quaternion_xyzw, gripper_abs)
     absolute command (metres, unit quaternion, 0 -1 gripper)

No Diffusion-Policy code inside: just call apply_action(action) at your control rate.

Requires:
    pip install bosdyn-client

Author: Moniruzzaman Akash
"""
from __future__ import annotations
import time, os, sys, signal
from pathlib import Path
import numpy as np

# ───── Spot SDK ─────────────────────────────────────────────────────────── #
from bosdyn.client           import create_standard_sdk
from bosdyn.client.lease     import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
from bosdyn.client.robot_command import RobotCommandBuilder, blocking_stand, blocking_sit, block_until_arm_arrives
from bosdyn.client.robot_state    import RobotStateClient
from bosdyn.client.estop   import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers  import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME
from bosdyn.client.docking           import DockingClient, blocking_dock_robot, blocking_undock, get_dock_id
from bosdyn.api              import geometry_pb2
from bosdyn.client.gripper_camera_param import GripperCameraParamClient
from bosdyn.client.image import ImageClient
from spot_images import SpotImages
from utils.spot_utils import quat_to_matrix, matrix_to_quat, rot6d_to_matrix, image_to_cv
import logging, cv2


# ───── Controller class ─────────────────────────────────────────────────── #
class SpotRobotController:
    def __init__(self,
                 robot_ip: str,
                 username: str,
                 password: str,
                 frame_name: str = BODY_FRAME_NAME,
                 default_cmd_time: float = 0.2):
        """
        frame_name : reference frame for arm pose commands ("vision" works well).
        default_cmd_time : seconds over which Spot will execute each pose command.
        """
        self.frame_name = frame_name
        self.t_exec     = default_cmd_time

        # --- connect & auth ------------------------------------------------
        sdk   = create_standard_sdk("spot-controller")
        self.robot = sdk.create_robot(robot_ip)
        self.robot.authenticate(username, password)
        self.robot.time_sync.wait_for_sync()

        # --- lease ---------------------------------------------------------
        lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        try:
            lease = lease_client.acquire()
        except ResourceAlreadyClaimedError:
            lease = lease_client.take()
        self.robot.lease_wallet.add(lease)
        self._lease_keepalive = LeaseKeepAlive(lease_client)

        # --- estop ---------------------------------------------------------
        estop_client = self.robot.ensure_client("estop")
        self.estop_endpoint = EstopEndpoint(estop_client, "vr_teleop", 9.0)
        self.estop_endpoint.force_simple_setup()
        self.estop_keepalive = EstopKeepAlive(self.estop_endpoint)
        self.estop_keepalive.allow()

        # --- clients -------------------------------------------------------
        self._cmd  = self.robot.ensure_client("robot-command")
        self._state= self.robot.ensure_client(RobotStateClient.default_service_name)

        # --- spot image -------------------------------------------------
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.gripper_cam_param_client = self.robot.ensure_client(GripperCameraParamClient.default_service_name)
        self.logger = logging.getLogger()
        self.spot_images = SpotImages(
            self.robot,
            self.logger,
            self.image_client,
            self.gripper_cam_param_client
        )

        # power on if needed
        if not self.robot.is_powered_on():
            print("> Powering on Spot ...")
            self.robot.power_on(timeout_sec=20)
        
        self.dock_id = get_dock_id(self.robot)

        signal.signal(signal.SIGINT, self._clean_shutdown)

    # ─── sensing ────────────────────────────────────────────────────────
    def _current_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (pos[3], quat[4]) of hand in self.frame_name."""
        snap = self._state.get_robot_state().kinematic_state.transforms_snapshot
        pose = get_a_tform_b(snap, self.frame_name, "hand")
        pos  = np.array([pose.x, pose.y, pose.z], dtype=np.float32)
        quat = np.array([pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w],
                        dtype=np.float32)
        return pos, quat

    def _current_gripper(self) -> float:
        man = self._state.get_robot_state().manipulator_state
        return float(man.gripper_open_percentage)

    def _clean_shutdown(self, *_):
        print("\n[!] Shutting down VR teleop ...")
        try:
            blocking_sit(self._cmd, timeout_sec=10)
        finally:
            self.estop_keepalive._end_periodic_check_in()
            self.estop_keepalive.stop()
            sys.exit(0)

    # ─── actuation helpers ───────────────────────────────────────────────
    def _send_arm_pose(self, pos: np.ndarray, quat: np.ndarray):
        """
        pos : (3,) np.ndarray - metres in self.frame_name
        quat: (4,) np.ndarray - unit quaternion (x,y,z,w)
        """
        pose_pb = geometry_pb2.SE3Pose(
            position = geometry_pb2.Vec3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
            rotation = geometry_pb2.Quaternion(x=float(quat[0]), y=float(quat[1]),
                                               z=float(quat[2]), w=float(quat[3])))
        arm_cmd = RobotCommandBuilder.arm_pose_command_from_pose(
            hand_pose = pose_pb,
            frame_name= self.frame_name,
            seconds   = self.t_exec)
        self._cmd.robot_command(arm_cmd, end_time_secs = time.time() + self.t_exec)

    def _send_gripper(self, opening: float):
        opening = opening / 100.0 if opening > 1.0 else opening # convert percentage to fraction if needed
        opening = float(np.clip(opening, 0.0, 1.0))
        grip_cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(opening)
        self._cmd.robot_command(grip_cmd)

    # ─── public API ──────────────────────────────────────────────────────
    def apply_action(self, action: np.ndarray):
        """
        Parameters
        ----------
        action : np.ndarray shape (10,)
            [ Δx Δy Δz , rot6d(6) , gripper_abs ]
        """
        if action.shape != (10,):
            raise ValueError("Action must be np.ndarray with shape (10,)")

        pos, quat = self._current_pose()

        delta_p   = action[:3]
        rot6d     = action[3:9]
        g_target  = float(action[9])

        # --- pose integration ------------------------------------------
        pos_cmd   = pos + delta_p
        R_curr    = quat_to_matrix(quat)
        R_rel     = rot6d_to_matrix(rot6d)
        R_cmd     = R_curr @ R_rel
        quat_cmd  = matrix_to_quat(R_cmd)

        # --- send to robot ---------------------------------------------
        print(f"Position({pos_cmd}), Quat({quat_cmd})")
        self._send_arm_pose(pos_cmd, quat_cmd)
        self._send_gripper(g_target)

    def move_arm_to(self,
                  pos_xyz: np.ndarray,
                  quat_xyzw: np.ndarray,
                  gripper: float):
        """
        Absolute command interface (no deltas).

        Parameters
        ----------
        pos_xyz      : (3,) np.ndarray  - metres in self.frame_name
        quat_xyzw    : (4,) np.ndarray  - unit quaternion
        gripper      : float            - 0.0 closed … 1.0 open
        """
        self._send_arm_pose(pos_xyz.astype(np.float32),
                            quat_xyzw.astype(np.float32))
        self._send_gripper(gripper)

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
                blocking_stand(self._cmd, timeout_sec=10)
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
        
    def get_hand_image(self) -> np.ndarray:
        """
        Get the image from the gripper camera.
        Returns
        -------
        np.ndarray
            The image from the gripper camera.
        """

        frame = image_to_cv(self.spot_images.get_hand_rgb_image())

        return frame.copy()
    
    def stow_arm(self):
        """Stow the arm to a safe position."""
        print("Stowing arm...")
        try:
            stow_cmd = RobotCommandBuilder.arm_stow_command()
            cmd_id = self._cmd.robot_command(stow_cmd, end_time_secs=time.time() + 5.0)
            block_until_arm_arrives(self._cmd, cmd_id)
            self.stowed = True
            print("Arm stowed.")
        except Exception as e:
            print(f"[!] Error stowing arm: {e}")

    def unstow_arm(self):
        """Unstow the arm to a ready position."""
        print("Unstowing arm...")
        try:
            unstow_cmd = RobotCommandBuilder.arm_ready_command()
            cmd_id = self._cmd.robot_command(unstow_cmd, end_time_secs=time.time() + 5.0)
            block_until_arm_arrives(self._cmd, cmd_id)
            self.stowed = False
            print("Arm unstowed.")
        except Exception as e:
            print(f"[!] Error unstowing arm: {e}")

