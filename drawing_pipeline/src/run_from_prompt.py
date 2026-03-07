import sys
import os
import subprocess
import torch
from primitives import sample_program, apply_params
from templates import TEMPLATES
from ik_solver import solve_trajectory, export_csv
from train import LabelToParams, LABEL_TO_IDX, PARAM_BOUNDS, PARAM_KEYS

DS_MM = 2.0
DT = 0.002


def parse_label(prompt):
    """Extract a template label from a free-form prompt."""
    prompt_lower = prompt.lower()
    for label in TEMPLATES:
        if label in prompt_lower:
            return label
    # try common aliases
    aliases = {"smiley": "face_simple", "face": "face_simple", "bicycle": "bike"}
    for alias, label in aliases.items():
        if alias in prompt_lower:
            return label
    raise ValueError(f"Could not find a known template in: '{prompt}'\n"
                     f"Known templates: {list(TEMPLATES.keys())}")


def predict_params(label):
    """Predict drawing params using the trained model, or defaults if no model."""
    model_path = os.path.join(os.path.dirname(__file__), "model.pt")
    if not os.path.exists(model_path):
        print("  No model.pt found, using defaults.")
        return {"scale": 1.0, "rot_deg": 0.0, "dx_mm": 0.0, "dy_mm": 0.0}

    checkpoint = torch.load(model_path, weights_only=False)
    label_to_idx = checkpoint["label_to_idx"]
    param_bounds = checkpoint["param_bounds"]

    model = LabelToParams(n_labels=len(label_to_idx))
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    idx = torch.tensor([label_to_idx[label]])
    with torch.no_grad():
        raw = model(idx).squeeze(0)

    params = {}
    for i, key in enumerate(PARAM_KEYS):
        lo, hi = param_bounds[key]
        params[key] = raw[i].item() * (hi - lo) + lo
    return params


def run_pipeline(label, params, csv_name=None):
    """Run template -> waypoints -> IK -> CSV.

    Returns the path to the generated CSV file.
    """
    if csv_name is None:
        csv_name = label

    program = TEMPLATES[label]()
    x_mm, y_mm, pen = sample_program(program, ds_mm=DS_MM)
    x_mm, y_mm = apply_params(x_mm, y_mm, **params)

    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0

    print(f"Running IK for '{label}' ({len(x_m)} waypoints) ...")
    thetaAll, min_det, success = solve_trajectory(x_m, y_m, pen)

    if not success:
        raise RuntimeError("IK failed. Try smaller scale or narrower params.")

    print(f"  IK done. min |det(Jb)| = {min_det:.6f}")

    # save to csv_controller's csv_files directory
    csv_dir = os.path.join(os.path.dirname(__file__), "..", "..", "csv_files")
    csv_dir = os.path.abspath(csv_dir)
    os.makedirs(csv_dir, exist_ok=True)

    csv_path = os.path.join(csv_dir, f"{csv_name}.csv")
    export_csv(csv_path, thetaAll, pen, dt=DT)
    print(f"  Saved: {csv_path}")

    return csv_path, csv_name


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} \"draw a plane\"")
        sys.exit(1)

    prompt = " ".join(sys.argv[1:])
    print(f"Prompt: \"{prompt}\"")

    label = parse_label(prompt)
    print(f"Matched template: {label}")

    params = predict_params(label)
    print(f"Params: {params}")

    csv_path, csv_name = run_pipeline(label, params)

    print(f"\nLaunching robot...")
    subprocess.run([
        "ros2", "launch", "csv_controller", "ur3e_csv.launch.py",
        f"csv_name:={csv_name}"
    ])


if __name__ == "__main__":
    main()
