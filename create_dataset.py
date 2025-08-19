'''
This script is used to create hf datasete from demos/*.npz files

Usage:
python3 create_dataset.py <dataset file name>

Author: Moniruzzaman Akash
'''

import numpy as np
import h5py
from pathlib import Path
import sys, os


# ---------- helpers ----------------------------------------------------------
def quat_to_matrix(q):
    """q: (...,4) with (x,y,z,w). Return (...,3,3)."""
    x, y, z, w = [q[..., i] for i in range(4)]
    tx, ty, tz = 2*x, 2*y, 2*z
    R = np.empty(q.shape[:-1] + (3, 3), dtype=q.dtype)
    R[..., 0, 0] = 1 - ty*y - tz*z
    R[..., 0, 1] = tx*y - tz*w
    R[..., 0, 2] = tx*z + ty*w
    R[..., 1, 0] = tx*y + tz*w
    R[..., 1, 1] = 1 - tx*x - tz*z
    R[..., 1, 2] = ty*z - tx*w
    R[..., 2, 0] = tx*z - ty*w
    R[..., 2, 1] = ty*z + tx*w
    R[..., 2, 2] = 1 - tx*x - ty*y
    return R

def mat_to_rot6d(R):
    """
    Convert a rotation matrix to Zhou et al.'s 6-D representation
    by *column-wise* stacking of the first two columns.
    R : (...,3,3)  →  (...,6)
    """
    return np.concatenate([R[..., :, 0],   # first column  (R00,R10,R20)
                           R[..., :, 1]],  # second column (R01,R11,R21)
                          axis=-1)

# ---------- main conversion --------------------------------------------------
def build_hdf5_from_npz(demos_dir: Path, h5_path: Path):
    with h5py.File(h5_path, "w") as hf:
        data_grp = hf.create_group("data")
        for npz_path in sorted(demos_dir.glob("*.npz")):
            demo_id = npz_path.stem
            
            grp     = data_grp.create_group(f"demo_{demo_id}")

            data    = np.load(npz_path, allow_pickle=True)

            # ------------------------------------------------------------------
            # OBSERVATIONS
            # ------------------------------------------------------------------
            obs_grp   = grp.create_group("obs")

            # images -----------------------------------------------------------
            frames = [img.astype(np.uint8) for img in data["images_0"]]
            frames = np.stack(frames, axis=0)                # (N,H,W,3)
            obs_grp.create_dataset("images_0", data=frames[:-1], compression="gzip")

            # split pose -------------------------------------------------------
            eef_pose   = data["ee_pose"].astype(np.float32)  # (N,7)
            eef_pos    = eef_pose[:, :3]                     # (N,3)
            eef_quat   = eef_pose[:, 3:]                     # (N,4)

            obs_grp.create_dataset("eef_pos",  data=eef_pos[:-1],  compression="gzip")
            obs_grp.create_dataset("eef_quat", data=eef_quat[:-1], compression="gzip")

            # copy other arrays (keep T‑1 length) ------------------------------
            for key in ("arm_q", "arm_dq", "vision_in_body",
                        "body_vel", "gripper", "ee_force", "t"):
                if key in data.files:
                    if key == "t":
                        arr = data[key].astype(np.float64)
                    else:   # other arrays are float32
                        arr = data[key].astype(np.float32)
                    obs_grp.create_dataset(key, data=arr[:-1], compression="gzip")
                else:
                    print(f"[WARN] {npz_path.name} missing '{key}'")

            # ------------------------------------------------------------------
            # ACTIONS      Δp  (3) + rot6d  (6) + Δgripper (1)  →  (N‑1,10)
            # ------------------------------------------------------------------
            pos_next     = eef_pos[1:]                      # (N-1,3)
            pos_curr     = eef_pos[:-1]
            delta_pos    = pos_next - pos_curr              # (N-1,3)

            R_curr       = quat_to_matrix(eef_quat[:-1])    # (N-1,3,3)
            R_next       = quat_to_matrix(eef_quat[1:])
            R_rel        = np.einsum("...ji,...jk->...ik", R_curr, R_next)
            rot6d        = mat_to_rot6d(R_rel).astype(np.float32)  # (N-1,6)

            # ----------- ABSOLUTE gripper command (take the *next* value) -----
            if "gripper" in data.files:
                g_target = data["gripper"][1:].astype(np.float32).reshape(-1, 1)  # (N‑1,1)
            else:
                g_target = np.zeros((delta_pos.shape[0], 1), dtype=np.float32)

            actions      = np.concatenate([delta_pos, rot6d, g_target], axis=-1)
            grp.create_dataset("actions", data=actions, compression="gzip")

            print(f"Packed demo_{demo_id}: obs {actions.shape[0]} steps, "
                  f"action shape {actions.shape}")

        # ----------------------------------------------------------------------
        # GLOBAL METADATA (optional)
        # ----------------------------------------------------------------------
        if "arm_joint_names" in data.files:
            names = np.array(data["arm_joint_names"].tolist(), dtype="S")
            hf.create_dataset("arm_joint_names", data=names, compression="gzip")

# ----------------------------------------------------------------------
if __name__ == "__main__":
    demos_folder = Path("demos")   # folder with *.npz files
    # get deemo output file name from arguments or use default
    if len(sys.argv) > 1:
        output_h5 = Path(sys.argv[1])
    else:
        output_h5 = Path("demos.h5")

    if os.path.exists(output_h5):
        print(f"Output file {output_h5} already exists. Please use different name.")
        sys.exit(1)

    build_hdf5_from_npz(demos_folder, output_h5)
    print(f"\nAll demos written to {output_h5}")
