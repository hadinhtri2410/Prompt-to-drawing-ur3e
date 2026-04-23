import os
import json
import numpy as np
from primitives import sample_program, apply_params
from templates import TEMPLATES
from ik_solver import solve_trajectory, export_csv

# parameter sampling bounds
PARAM_BOUNDS = {
    "scale":   (0.6, 1.5),
    "rot_deg": (-45.0, 45.0),
    "dx_mm":   (-30.0, 30.0),
    "dy_mm":   (-30.0, 30.0),
}

# rejection thresholds
MIN_DET_THRESHOLD = 1e-4
MAX_JOINT_JUMP = 0.3

DS_MM = 2.0
DT = 0.002


def random_params():
    return {
        "scale":   np.random.uniform(*PARAM_BOUNDS["scale"]),
        "rot_deg": np.random.uniform(*PARAM_BOUNDS["rot_deg"]),
        "dx_mm":   np.random.uniform(*PARAM_BOUNDS["dx_mm"]),
        "dy_mm":   np.random.uniform(*PARAM_BOUNDS["dy_mm"]),
    }


def generate_one(label, template_fn, params):
    """Run the full pipeline for one sample. Returns (thetaAll, pen, success)."""
    program = template_fn()
    x_mm, y_mm, pen = sample_program(program, ds_mm=DS_MM)
    x_mm, y_mm = apply_params(x_mm, y_mm, **params)

    # convert mm to meters
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0

    thetaAll, min_det, success = solve_trajectory(x_m, y_m, pen)
    if not success:
        return None, None, "ik_failed", 0.0

    if min_det < MIN_DET_THRESHOLD:
        return None, None, "singularity", 0.0

    # check for elbow flips (large joint jumps between consecutive waypoints)
    max_jump = np.max(np.abs(np.diff(thetaAll, axis=1)))
    if max_jump > MAX_JOINT_JUMP:
        return None, None, "elbow_flip", 0.0

    return thetaAll, pen, "ok", min_det


def generate_dataset(out_dir, n_per_label=300, max_attempts_factor=3):
    """Generate training data for all templates.

    Args:
        out_dir: output directory (e.g. "data/train")
        n_per_label: target number of valid samples per label
        max_attempts_factor: give up after n_per_label * this many attempts
    """
    os.makedirs(out_dir, exist_ok=True)

    for label, template_fn in TEMPLATES.items():
        count = 0
        attempts = 0
        max_attempts = n_per_label * max_attempts_factor

        print(f"Generating '{label}' ...")

        while count < n_per_label and attempts < max_attempts:
            attempts += 1
            params = random_params()
            thetaAll, pen, status, min_det = generate_one(label, template_fn, params)

            if status != "ok":
                continue

            name = f"{label}_{count:04d}"

            # save metadata (include min_det so training can filter by quality)
            meta = {"label": label, "ds_mm": DS_MM, "min_det": float(min_det), **params}
            with open(os.path.join(out_dir, f"{name}.json"), "w") as f:
                json.dump(meta, f, indent=2)

            # save joint trajectory CSV
            export_csv(os.path.join(out_dir, f"{name}.csv"), thetaAll, pen, dt=DT)

            count += 1

        rejected = attempts - count
        print(f"  {label}: {count}/{n_per_label} generated, "
              f"{rejected} rejected ({100*rejected/max(attempts,1):.0f}%)")

        if count < n_per_label:
            print(f"  WARNING: only got {count}, consider narrowing param bounds")


if __name__ == "__main__":
    generate_dataset("data/train", n_per_label=300)
    generate_dataset("data/val",   n_per_label=50)
    generate_dataset("data/test",  n_per_label=50)
