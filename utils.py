import numpy as np
import math
from bosdyn.api import geometry_pb2
from bosdyn.client.math_helpers  import SE3Pose, Quat

def rot_to_quat(R: np.ndarray) -> Quat:
    """Convert 3×3 rotation matrix to bosdyn Quat."""
    t = np.trace(R)
    if t > 0.0:
        S = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return Quat(w=w, x=x, y=y, z=z)

def mat_to_se3(mat: np.ndarray) -> SE3Pose:
    """4×4 numpy SE(3) → bosdyn SE3Pose (SDK 5.x signature)."""
    pos  = mat[:3, 3]
    quat = rot_to_quat(mat[:3, :3])
    return SE3Pose(pos[0], pos[1], pos[2], quat)   # ← explicit x, y, z


def reaxis(mat4: np.ndarray) -> np.ndarray:
    """
    Rotate pose from controller frame to robot-hand frame.

    Controller-frame  →  Desired-hand-frame
    new x  (forward)  = −old y
    new y  (left)     = −old x
    new z  (up)       = -old z

    in 'xyz' convention, x=-180, y=0, z=90
    which is 
    R_CONV = np.array([
        [ 0, -1,  0],
        [-1,  0,  0],
        [ 0,  0, -1]])

    But we calculate for x=-150, y=0, z=90(arm frame is slightly rotated along x-axis)
    """
    # right-handed, det = +1    
    R_CONV = np.array([
        [ 0, -1,  0],
        [-0.8660254, -0.0000000,  0.5000000],
        [ -0.5000000, -0.0000000, -0.8660254]])

    m = mat4.copy()
    # rotate orientation
    m[:3, :3] = mat4[:3, :3] @ R_CONV
    # no need to rotate position, since it is in the same frame as orientation
    
    return m