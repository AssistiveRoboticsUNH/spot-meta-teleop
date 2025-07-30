#!/usr/bin/env python3
"""
demo_recorder.py  –  Continuous Spot logger for diffusion-policy demos
---------------------------------------------------------------------
• Records at `fps` Hz from `start()` until `stop()`
• Dumps one compressed NPZ per session: images, joint states, vision←body pose, … 
• Optionally previews the hand camera while running
---------------------------------------------------------------------
Quick use inside control_spot.py
---------------------------------------------------------------------
from demo_recorder import DemoRecorder
...
self.recorder = DemoRecorder(
        robot=self.robot,
        spot_images=self.spot_images,
        state_client=self.state_client,
        out_dir="demos",
        fps=5,
        preview=True)
self.recorder.start()

# … tele-op loop …

def _clean_shutdown(self, *a):
    self.recorder.stop()
    ...
"""
from __future__ import annotations
import time, threading, subprocess
import numpy as np
import cv2
from pathlib import Path
from typing import Dict, Optional, List

from bosdyn.api import image_pb2, geometry_pb2
from bosdyn.client import math_helpers
from utils.spot_utils import pose_to_vec, frame_pose, image_to_cv


def play_sound(path: str, block = False):
    # -q = quiet, removes console output; omit if you want to see mpg123 logs
    player = subprocess.Popen(["mpg123", "-q", "media/start.mp3"])
    if block:
        player.wait()

# ------------------------------------------------------------------ #
#  Recorder                                                          #
# ------------------------------------------------------------------ #

class DemoRecorder:
    def __init__(
        self,
        robot,
        spot_images,
        state_client,
        out_dir: str = "demos",
        fps: float = 10.0,
        preview: bool = False,
    ):
        self.robot         = robot
        self.spot_images   = spot_images
        self.state_client  = state_client
        self.fps           = fps
        self.preview       = preview

        self._latest_frame = None
        self._frame_lock = threading.Lock()


        self.out_dir       = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        # thread plumbing
        self._stop_evt     = threading.Event()
        self._thread       = threading.Thread(target=self._worker, daemon=True)

        # data buffers (grow until stop())
        self._frames:      List[np.ndarray]     = []
        self._state_bufs:  Dict[str, List[np.ndarray]] = {}
        self._joint_names: Optional[np.ndarray] = None

        self.is_recording = False

    # ---------------- public control ------------------------------------ #
    def start(self):
        if self._thread.is_alive():
            print("[DemoRecorder] Already running.")
            try:
                play_sound('media/denied.mp3')
            except Exception as e:
                print(f"[DemoRecorder] Play sound error: {e}")
            return
        # clear old buffers
        self._frames.clear()
        self._state_bufs.clear()
        self._joint_names = None
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        try:
            play_sound('media/start.mp3', block=True)
        except Exception as e:
                print(f"[DemoRecorder] Play sound error: {e}")
        self._thread.start()

        self.is_recording = True
        print("[DemoRecorder] Started recording.")
        

    def stop(self):
        print("[DemoRecorder] Stopping...")
        self._stop_evt.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            print("[DemoRecorder] WARNING: worker did not exit—forcing shutdown.")
        try:
            play_sound('media/stop.mp3')
        except Exception as e:
                print(f"[DemoRecorder] Play sound error: {e}")

        self._flush_to_disk()
        self.is_recording = False
        print("[DemoRecorder] Stopped and saved session.")
        try:
            play_sound('media/saved.mp3')
        except Exception as e:
                print(f"[DemoRecorder] Play sound error: {e}")

    def poll_preview(self):
        """Call this periodically from the main thread to show the latest frame."""
        if not self.preview:
            return

        if not self.is_recording:
            try:
                cv2.destroyAllWindows()
            except:
                pass

        frame = None
        with self._frame_lock:
            if self._latest_frame is not None:
                frame = self._latest_frame.copy()

        if frame is not None:
            cv2.imshow("Hand RGB", frame)
            cv2.waitKey(1)  # Allows OpenCV to process UI events

    # ---------------- background thread --------------------------------- #
    def _worker(self):
        period = 1.0 / self.fps
        next_t = time.time()
        while not self._stop_evt.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
                continue
            next_t += period
            try:
                frame = image_to_cv(self.spot_images.get_hand_rgb_image())
                state = self.state_client.get_robot_state()
                self._frames.append(frame)
                vecs  = self._extract_state(state)
                for k, v in vecs.items():
                    self._state_bufs.setdefault(k, []).append(v)
                # save for preview
                if self.preview:
                    with self._frame_lock:
                        self._latest_frame = frame.copy()
            except Exception as e:
                print(f"[DemoRecorder] {e}")

    # ---------------- state extraction ---------------------------------- #
    def _extract_state(self, state) -> Dict[str, np.ndarray]:
        kin       = state.kinematic_state
        snapshot  = kin.transforms_snapshot

        # 1. --- Arm joint angles + velocities --------------------------------------------------
        q, dq, names = [], [], []
        for j in kin.joint_states:
            if j.name.startswith("arm0."):
                names.append(j.name)
                q.append(j.position.value)
                dq.append(j.velocity.value)

        if self._joint_names is None:
            self._joint_names = np.array(names)

        # 2. Hand pose in body frame
        hand_pose = frame_pose(snapshot, "hand")
        hand_vec = pose_to_vec(hand_pose) if hand_pose else np.zeros(7, np.float32)

        # --- vision←body pose -------------------------------------------
        vision_in_body = frame_pose(snapshot, "vision")     # vision expressed in body frame
        if vision_in_body:
            vis_vec = pose_to_vec(vision_in_body)
        else:
            vis_vec = np.zeros(7, np.float32)

        # --- body velocity (vision) -------------------------------------
        vlin = kin.velocity_of_body_in_vision.linear
        vang = kin.velocity_of_body_in_vision.angular
        body_vel = np.array([vlin.x, vlin.y, vlin.z,
                             vang.x, vang.y, vang.z], dtype=np.float32)

        # --- gripper & wrench -------------------------------------------
        man      = state.manipulator_state
        gripper  = np.array([man.gripper_open_percentage/100.0], dtype=np.float32)
        wrench   = np.array([
              man.estimated_end_effector_force_in_hand.x,
              man.estimated_end_effector_force_in_hand.y,
              man.estimated_end_effector_force_in_hand.z], dtype=np.float32)

        # --- timestamp ----------------------------------------
        ts    = kin.acquisition_timestamp.seconds + \
                kin.acquisition_timestamp.nanos * 1e-9

        return dict(
            arm_q = np.array(q,  dtype=np.float32),
            arm_dq= np.array(dq, dtype=np.float32),
            ee_pose = hand_vec,
            ee_force = wrench,
            gripper  = gripper,
            vision_in_body = vis_vec,
            body_vel = body_vel,
            t        = np.array([ts],   dtype=np.float64),
        )

    # ---------------- disk writer -------------------------------------- #
    def _flush_to_disk(self):
        if not self._frames:
            print("[DemoRecorder] Nothing captured - no file written.")
            return

        # build session dict
        session = {
            "images_0" : np.array(self._frames, dtype=object),   # ragged
            "arm_joint_names" : self._joint_names,
        }
        for k, lst in self._state_bufs.items():
            session[k] = np.stack(lst)

        # choose next chronological file name
        existing = list(self.out_dir.glob("*.npz"))
        nums     = [int(p.stem) for p in existing if p.stem.isdigit()]
        idx      = max(nums) + 1 if nums else 0
        fname    = self.out_dir / f"{idx}.npz"

        np.savez_compressed(fname, **session)
        print(f"[DemoRecorder] Wrote {fname}")

