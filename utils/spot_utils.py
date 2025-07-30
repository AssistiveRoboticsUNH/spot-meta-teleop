import numpy as np
import math
from bosdyn.api import geometry_pb2
from bosdyn.client.math_helpers  import SE3Pose, Quat
import cv2
from bosdyn.api import image_pb2

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
    """4x4 numpy SE(3) → bosdyn SE3Pose (SDK 5.x signature)."""
    pos  = mat[:3, 3]
    quat = rot_to_quat(mat[:3, :3])
    return SE3Pose(pos[0], pos[1], pos[2], quat)   # ← explicit tx, ty, tz , w, x, y, z


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

def proto_to_cv2(img_resp):
    """Convert a Spot ImageResponse with PIXEL_FORMAT_RGB_U8 to an OpenCV BGR image."""
    rows   = img_resp.shot.image.rows
    cols   = img_resp.shot.image.cols
    data   = np.frombuffer(img_resp.shot.image.data, dtype=np.uint8)
    rgb    = data.reshape((rows, cols, 3))              # still RGB
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)         # OpenCV wants BGR

def image_to_cv(img_resp: image_pb2.ImageResponse) -> np.ndarray:
    """Robust Spot ImageResponse → OpenCV ndarray (BGR or GRAY)."""
    img = img_resp.shot.image
    fmt = img_resp.shot.image.format
    rows, cols = img.rows, img.cols

    # --- 1. JPEG ---------------------------------------------------------
    if fmt == image_pb2.Image.FORMAT_JPEG:
        buf = np.frombuffer(img.data, dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)       # already BGR

    # --- 2. RAW ----------------------------------------------------------
    # Size of the data tells us how many channels we have.
    chan = len(img.data) // (rows * cols)

    if img.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        # 16-bit depth
        depth = np.frombuffer(img.data, dtype=np.uint16).reshape(rows, cols)
        return depth                                      # caller decides how to visualise

    if chan == 1:
        gray = np.frombuffer(img.data, dtype=np.uint8).reshape(rows, cols)
        # if you want colourised output for cv2.imshow, convert once here
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if chan == 3:
        rgb = np.frombuffer(img.data, dtype=np.uint8).reshape(rows, cols, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    raise ValueError(f"Unhandled channel count ({chan}) or pixel_format {img.pixel_format}")