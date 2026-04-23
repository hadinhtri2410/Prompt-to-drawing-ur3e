"""
Drawing quality evaluation.

For each shape template, this script:
  1. Generates the intended 2D waypoints (ground truth) from the template
  2. Loads the generated CSV, runs FK on every joint row to recover the
     actual end-effector path, and projects it back to the drawing plane
  3. Computes three metrics between intended and actual (pen-down only):
       - Mean distance error (mm)   -- average nearest-neighbour distance
       - Hausdorff distance (mm)    -- worst-case deviation
       - Rasterized IoU             -- pixel-level shape overlap
  4. Declares success if mean error < SUCCESS_THRESHOLD_MM
  5. Prints a summary table and saves overlay PNGs

Usage:
    # First generate a CSV for each shape:
    python run_from_prompt.py "draw a plane"
    python run_from_prompt.py "draw a bike"
    ... etc.

    # Then evaluate:
    python eval_drawing_quality.py
"""

import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

from primitives import sample_program, apply_params
from templates import TEMPLATES
from ik_solver import (ECE569_FKinSpace, ECE569_TransInv,
                       M_HOME, S_AXES, THETA0)

# ── thresholds ────────────────────────────────────────────────────────────────
SUCCESS_THRESHOLD_MM = 5.0   # mean error below this → success
DS_MM                = 2.0   # must match what was used to generate the CSV
RASTER_SIZE          = 256   # pixels for IoU rasterisation

CSV_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "csv_files")
)


# ── helpers ───────────────────────────────────────────────────────────────────

def load_csv_thetas(path):
    """Return (N, 6) joint-angle array from a trajectory CSV (cols 1-6)."""
    data = np.genfromtxt(path, delimiter=',')
    if data.ndim == 1:
        data = data[np.newaxis, :]
    return data[:, 1:7]          # columns: t, q1..q6, pen


def load_csv_pen(path):
    """Return (N,) pen array from a trajectory CSV (col 7)."""
    data = np.genfromtxt(path, delimiter=',')
    if data.ndim == 1:
        data = data[np.newaxis, :]
    return data[:, 7].astype(int) if data.shape[1] > 7 else np.ones(len(data), dtype=int)


def fk_to_drawing_plane(thetas):
    """
    Run FK on each row of thetas and project back to the 2D drawing plane.

    The drawing plane is defined as the XY plane of T0 = FK(THETA0).
    solve_trajectory places each waypoint at  T0 @ [[I, [x,y,0]], [0,1]],
    so we recover (x, y) by computing  T0^{-1} @ FK(theta_i)  and reading
    the translation's first two components.

    Returns: (N, 2) array in metres.
    """
    T0     = ECE569_FKinSpace(M_HOME, S_AXES, THETA0)
    T0_inv = ECE569_TransInv(T0)
    xy = np.zeros((len(thetas), 2))
    for i, theta in enumerate(thetas):
        T = ECE569_FKinSpace(M_HOME, S_AXES, theta)
        T_rel = T0_inv @ T
        xy[i] = T_rel[:2, 3]   # x, y offset in metres
    return xy


def chamfer_and_hausdorff(pts_a, pts_b):
    """
    pts_a, pts_b: (N,2) and (M,2) arrays in the same units.
    Returns (mean_dist, hausdorff_dist).
    mean_dist  = 0.5*(mean of min-dists A→B + mean of min-dists B→A)
    hausdorff  = max(max min-dists A→B, max min-dists B→A)
    """
    tree_a = cKDTree(pts_a)
    tree_b = cKDTree(pts_b)
    d_ab, _ = tree_b.query(pts_a)
    d_ba, _ = tree_a.query(pts_b)
    mean_dist  = 0.5 * (d_ab.mean() + d_ba.mean())
    hausdorff  = max(d_ab.max(), d_ba.max())
    return mean_dist, hausdorff


def rasterize(pts, size=RASTER_SIZE, margin=0.1):
    """
    Rasterize a set of 2D points into a binary image of shape (size, size).
    pts: (N,2) array.  margin: fractional border.
    """
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    span = (hi - lo).max()
    if span < 1e-9:
        span = 1.0
    scale  = (1.0 - 2 * margin) * size / span
    offset = margin * size - lo * scale
    img = np.zeros((size, size), dtype=bool)
    px = (pts * scale + offset).astype(int)
    px = np.clip(px, 0, size - 1)
    img[px[:, 1], px[:, 0]] = True
    return img


def iou(img_a, img_b):
    intersection = (img_a & img_b).sum()
    union        = (img_a | img_b).sum()
    return intersection / union if union > 0 else 0.0


def save_overlay(label, gt_mm, actual_mm, mean_err, out_dir):
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.plot(gt_mm[:, 0],     gt_mm[:, 1],     'b-',  lw=1.5, label='intended')
    ax.plot(actual_mm[:, 0], actual_mm[:, 1], 'r--', lw=1.5, label='actual FK')
    ax.set_aspect('equal')
    ax.set_title(f"{label}  mean error={mean_err:.2f} mm")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.4)
    path = os.path.join(out_dir, f"{label}_overlay.png")
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    return path


# ── main evaluation ───────────────────────────────────────────────────────────

def evaluate_shape(label, template_fn, params=None):
    """
    Evaluate one shape.  params defaults to identity (no transform).
    Returns dict with keys: mean_mm, hausdorff_mm, iou, success, error_msg.
    """
    if params is None:
        params = {"scale": 1.0, "rot_deg": 0.0, "dx_mm": 0.0, "dy_mm": 0.0}

    csv_path = os.path.join(CSV_DIR, f"{label}.csv")
    if not os.path.exists(csv_path):
        return {"label": label, "mean_mm": None, "hausdorff_mm": None,
                "iou": None, "success": False,
                "error_msg": f"CSV not found: {csv_path}"}

    # ── ground truth ──
    program = template_fn()
    x_mm, y_mm, pen_gt = sample_program(program, ds_mm=DS_MM)
    x_mm, y_mm = apply_params(x_mm, y_mm, **params)
    gt_m   = np.column_stack([x_mm, y_mm]) / 1000.0
    mask_gt = pen_gt.astype(bool)

    # ── actual trajectory via FK ──
    thetas  = load_csv_thetas(csv_path)
    pen_act = load_csv_pen(csv_path)
    actual_m = fk_to_drawing_plane(thetas)
    mask_act = pen_act.astype(bool)

    # only compare pen-down segments
    gt_down     = gt_m[mask_gt]
    actual_down = actual_m[mask_act]

    if len(gt_down) == 0 or len(actual_down) == 0:
        return {"label": label, "mean_mm": None, "hausdorff_mm": None,
                "iou": None, "success": False,
                "error_msg": "no pen-down points"}

    mean_m, hd_m = chamfer_and_hausdorff(gt_down, actual_down)
    mean_mm = mean_m * 1000.0
    hd_mm   = hd_m   * 1000.0

    # rasterise both in the same coordinate frame
    all_pts = np.vstack([gt_down, actual_down])
    lo, hi  = all_pts.min(axis=0), all_pts.max(axis=0)

    def rasterize_shared(pts):
        span = (hi - lo).max()
        if span < 1e-9:
            span = 1.0
        margin = 0.05
        scale  = (1.0 - 2 * margin) * RASTER_SIZE / span
        offset = margin * RASTER_SIZE - lo * scale
        img = np.zeros((RASTER_SIZE, RASTER_SIZE), dtype=bool)
        px  = (pts * scale + offset).astype(int)
        px  = np.clip(px, 0, RASTER_SIZE - 1)
        # dilate slightly so thin curves register
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                img[np.clip(px[:, 1]+dy, 0, RASTER_SIZE-1),
                    np.clip(px[:, 0]+dx, 0, RASTER_SIZE-1)] = True
        return img

    img_gt  = rasterize_shared(gt_down)
    img_act = rasterize_shared(actual_down)
    shape_iou = iou(img_gt, img_act)

    return {
        "label":        label,
        "mean_mm":      mean_mm,
        "hausdorff_mm": hd_mm,
        "iou":          shape_iou,
        "success":      mean_mm < SUCCESS_THRESHOLD_MM,
        "gt_mm":        gt_down  * 1000.0,
        "actual_mm":    actual_down * 1000.0,
        "error_msg":    None,
    }


def run_evaluation():
    out_dir = os.path.dirname(__file__)
    results = []

    for label, template_fn in TEMPLATES.items():
        print(f"Evaluating '{label}' ...", end="  ", flush=True)
        r = evaluate_shape(label, template_fn)
        results.append(r)

        if r["error_msg"]:
            print(f"SKIPPED — {r['error_msg']}")
        else:
            print(f"mean={r['mean_mm']:.2f} mm  "
                  f"hausdorff={r['hausdorff_mm']:.2f} mm  "
                  f"IoU={r['iou']:.3f}  "
                  f"{'OK' if r['success'] else 'FAIL'}")
            save_overlay(label, r["gt_mm"], r["actual_mm"], r["mean_mm"], out_dir)

    # ── summary table ──
    valid = [r for r in results if r["error_msg"] is None]
    n_success = sum(r["success"] for r in valid)

    print("\n" + "=" * 70)
    print("DRAWING QUALITY SUMMARY")
    print(f"Success threshold: mean error < {SUCCESS_THRESHOLD_MM} mm")
    print("=" * 70)
    print(f"{'Shape':<15} {'Mean err (mm)':>14} {'Hausdorff (mm)':>15} {'IoU':>7} {'Pass?':>6}")
    print("-" * 70)
    for r in results:
        if r["error_msg"]:
            print(f"{r['label']:<15} {'— (no CSV)':>14}")
        else:
            status = "YES" if r["success"] else "NO"
            print(f"{r['label']:<15} {r['mean_mm']:>14.2f} "
                  f"{r['hausdorff_mm']:>15.2f} {r['iou']:>7.3f} {status:>6}")

    if valid:
        print("-" * 70)
        print(f"{'Overall':<15} "
              f"{np.mean([r['mean_mm'] for r in valid]):>14.2f} "
              f"{np.mean([r['hausdorff_mm'] for r in valid]):>15.2f} "
              f"{np.mean([r['iou'] for r in valid]):>7.3f} "
              f"{n_success}/{len(valid)}")
        print(f"\nSuccess rate: {n_success}/{len(valid)} "
              f"({100*n_success/len(valid):.0f}%)")

    print(f"\nOverlay images saved to: {out_dir}/<label>_overlay.png")


if __name__ == "__main__":
    run_evaluation()
