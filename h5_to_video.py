#!/usr/bin/env python3

'''
This script converts demonstration data stored in an HDF5 file into video files.
It allows users to select specific demonstration IDs to convert, and generates
MP4 videos and saves them in videos/<demo_file_name>/ directory.

Usage:
    python h5_to_video.py <demo_file.h5>
'''

import re
import sys
from pathlib import Path

import cv2
import h5py
import numpy as np


def parse_demo_selection(selection: str, available_ids: list[int]):
    sel = selection.strip().lower()
    if not sel or sel in {"all", "*"}:
        return "all"
    demo_ids = set()
    for part in sel.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            m = re.match(r"^(\d+)\s*-\s*(\d+)$", part)
            if not m:
                raise ValueError(f"Invalid range: '{part}'")
            start, end = int(m.group(1)), int(m.group(2))
            if start > end:
                start, end = end, start
            demo_ids.update(range(start, end + 1))
        else:
            if not part.isdigit():
                raise ValueError(f"Invalid demo id: '{part}'")
            demo_ids.add(int(part))
    demo_ids = sorted(demo_ids)
    missing = [d for d in demo_ids if d not in set(available_ids)]
    if missing:
        raise ValueError(f"Requested demo id(s) not available: {missing}")
    return demo_ids


def compute_fps(obs_grp, default_fps: float) -> float:
    if "t" not in obs_grp:
        return default_fps
    t = np.asarray(obs_grp["t"]).reshape(-1)
    if t.size <= 1:
        return default_fps
    dt = np.diff(t)
    dt = dt[dt > 0]
    if dt.size == 0:
        return default_fps
    return float(1.0 / np.mean(dt))


def render_demo_video(obs_grp, out_path: Path, default_fps: float) -> None:
    if "images_0" not in obs_grp:
        print(f"[WARN] {out_path.stem}: missing images_0")
        return
    frames = obs_grp["images_0"]
    if frames.shape[0] == 0:
        print(f"[WARN] {out_path.stem}: no frames")
        return

    fps = compute_fps(obs_grp, default_fps)
    h, w = frames[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))

    total = frames.shape[0]
    bar_width = 28
    for i in range(frames.shape[0]):
        frame = frames[i]
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)
        out.write(frame)
        if (i + 1) == total or (i + 1) % max(1, total // bar_width) == 0:
            filled = int((i + 1) / total * bar_width)
            bar = "=" * filled + " " * (bar_width - filled)
            pct = (i + 1) * 100 // total
            print(f"\r[{bar}] {pct:3d}% {out_path.stem}", end="", flush=True)
    out.release()
    print(f"\r[{'=' * bar_width}] 100% {out_path.stem}")
    print(f"Wrote {out_path.name} @ {fps:.1f} fps")


def main():
    if len(sys.argv) < 2:
        print("[ERROR] Demo file path is required. Usage: python3 h5_to_video.py <demo_file.h5>")
        return 1
    h5_path = sys.argv[1]
    h5_path = Path(h5_path)
    out_dir = h5_path.parent / "videos" / h5_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)

    with h5py.File(h5_path, "r") as hf:
        if "data" not in hf:
            print(f"[ERROR] Missing 'data' group in {h5_path}")
            return
        data_grp = hf["data"]

        available_ids = []
        for key in data_grp.keys():
            if key.startswith("demo_") and key[5:].isdigit():
                available_ids.append(int(key[5:]))
        available_ids.sort()

        if not available_ids:
            print("[WARN] No demos found.")
            return 0

        min_id = available_ids[0]
        max_id = available_ids[-1]
        print(f"Available demo id range: [{min_id},{max_id}]")
        print(f"Total demos: {len(available_ids)}")
        selection = input("Demo id(s) (e.g., 0 or 1-3 or 0,2,5) [Enter for all]: ").strip()

        try:
            demo_ids = parse_demo_selection(selection, available_ids)
        except ValueError as e:
            print(f"[ERROR] {e}")
            return 1
        if demo_ids == "all":
            demo_ids = available_ids

        if not demo_ids:
            print("[WARN] No demos selected.")
            return

        for demo_id in demo_ids:
            demo_key = f"demo_{demo_id}"
            if demo_key not in data_grp:
                print(f"[WARN] Missing {demo_key}")
                continue
            obs_grp = data_grp[demo_key].get("obs")
            if obs_grp is None:
                print(f"[WARN] {demo_key} missing obs group")
                continue
            out_path = out_dir / f"{demo_key}.mp4"
            render_demo_video(obs_grp, out_path, default_fps=10.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
