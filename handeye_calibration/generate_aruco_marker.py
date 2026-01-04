#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
import cv2

PAGE_SIZES_IN = {
    "a4": (8.27, 11.69),
    "letter": (8.5, 11.0),
}


def write_marker_pdf(out_path: Path, marker: np.ndarray, size_m: float, pixels: int, page_size_in):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise ImportError(
            "matplotlib is required to generate PDF output. "
            "Install it or choose a .png output path."
        ) from exc

    size_in = size_m * 39.3700787
    dpi = pixels / size_in
    page_w, page_h = page_size_in
    fig = plt.figure(figsize=(page_w, page_h))
    w_frac = size_in / page_w
    h_frac = size_in / page_h
    left = (1.0 - w_frac) / 2.0
    bottom = (1.0 - h_frac) / 2.0
    ax = fig.add_axes([left, bottom, w_frac, h_frac])
    ax.imshow(marker, cmap="gray", interpolation="nearest")
    ax.axis("off")
    fig.savefig(str(out_path), dpi=dpi)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Generate an ArUco marker image.")
    parser.add_argument("--marker-id", type=int, default=7, help="ArUco marker id")
    parser.add_argument("--dict-name", type=str, default="DICT_4X4_1000",
                        help="ArUco dictionary name")
    parser.add_argument("--size-m", type=float, default=0.186,
                        help="Physical marker size in meters")
    parser.add_argument("--pixels", type=int, default=1000,
                        help="Output image size in pixels")
    parser.add_argument("--out", type=Path, default=None,
                        help="Output path (default: handeye_calibration/aruco_<dict>_id<id>_<size>m.pdf)")
    args = parser.parse_args()

    aruco_dict_id = getattr(cv2.aruco, args.dict_name, None)
    if aruco_dict_id is None:
        raise ValueError(f"Unknown ArUco dict '{args.dict_name}'")
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

    if hasattr(cv2.aruco, "drawMarker"):
        marker = cv2.aruco.drawMarker(aruco_dict, args.marker_id, args.pixels)
    elif hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(aruco_dict, args.marker_id, args.pixels)
    else:
        raise AttributeError("cv2.aruco has no drawMarker/generateImageMarker. "
                             "Install opencv-contrib-python for ArUco support.")

    if args.out is None:
        base = Path(__file__).resolve().parent / (
            f"aruco_{args.dict_name.lower()}_id{args.marker_id}_{args.size_m:.3f}m"
        )
    else:
        base = Path(args.out)

    if base.suffix.lower() == ".png":
        base.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(base), marker)
        print(f"Wrote {base} ({args.pixels}x{args.pixels}px), size={args.size_m}m")
        return

    if base.suffix.lower() == ".pdf":
        base = base.with_suffix("")

    base.parent.mkdir(parents=True, exist_ok=True)
    for label, page_size_in in PAGE_SIZES_IN.items():
        out_path = base.parent / f"{base.name}_{label}.pdf"
        write_marker_pdf(out_path, marker, args.size_m, args.pixels, page_size_in)
        print(f"Wrote {out_path} (print at 100% scale), size={args.size_m}m")


if __name__ == "__main__":
    main()
