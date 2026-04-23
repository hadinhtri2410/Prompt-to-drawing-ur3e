"""
Evaluation script for label detection accuracy.

Measures:
  - Per-label accuracy (exact + alias prompts)
  - Overall accuracy
  - Confusion matrix (what label was predicted when wrong)
  - False-negative rate (prompts that raised ValueError)
  - IK convergence rate and rejection breakdown per shape
  - min|det J| statistics per shape (mean, std, min)
  - Embedding cosine similarity matrix between labels

Run:
    python eval_label_detection.py
"""

import os
import json
import glob
import numpy as np
from collections import defaultdict
from run_from_prompt import parse_label
from ik_solver import solve_trajectory, ECE569_JacobianBody, B_AXES
from primitives import sample_program, apply_params
from templates import TEMPLATES
from train import LABEL_TO_IDX, LabelToParams
import torch
import torch.nn.functional as F


# ---------------------------------------------------------------------------
# Test suite: (prompt, expected_label)
# Covers exact names, aliases, natural phrasings, negatives
# ---------------------------------------------------------------------------
TEST_PROMPTS = [
    # --- plane ---
    ("draw a plane",                     "plane"),
    ("can you draw a plane please",      "plane"),
    ("I want a plane",                   "plane"),
    ("draw me an airplane",              "plane"),
    ("plane outline",                    "plane"),

    # --- bike ---
    ("draw a bike",                      "bike"),
    ("draw a bicycle",                   "bike"),
    ("I'd like a bicycle please",        "bike"),
    ("can you draw a bike for me",       "bike"),
    ("sketch a bike",                    "bike"),

    # --- face_simple ---
    ("draw a face",                      "face_simple"),
    ("draw a smiley",                    "face_simple"),
    ("smiley face please",               "face_simple"),
    ("draw a face_simple",               "face_simple"),
    ("can you draw a smiley face",       "face_simple"),

    # --- square ---
    ("draw a square",                    "square"),
    ("draw a rectangle",                 "square"),
    ("draw a box",                       "square"),
    ("draw a rect",                      "square"),
    ("I want a square shape",            "square"),

    # --- circle ---
    ("draw a circle",                    "circle"),
    ("can you draw a circle",            "circle"),
    ("draw me a circle",                 "circle"),
    ("a circle please",                  "circle"),
    ("draw a big circle",                "circle"),

    # --- lissajous ---
    ("draw a lissajous",                 "lissajous"),
    ("draw a lissajous curve",           "lissajous"),
    ("draw a lissajous figure",          "lissajous"),
    ("lissajous pattern please",         "lissajous"),
    ("I want a lissajous",               "lissajous"),

    # --- should fail (unknown) ---
    ("draw a dog",                       None),
    ("draw a house",                     None),
    ("draw a star",                      None),
]


def run_label_detection():
    per_label_correct  = defaultdict(int)
    per_label_total    = defaultdict(int)
    confusion          = defaultdict(lambda: defaultdict(int))  # confusion[true][pred]
    false_negatives    = []  # prompts that raised ValueError unexpectedly
    false_positives    = []  # prompts that should have failed but returned a label

    for prompt, expected in TEST_PROMPTS:
        per_label_total[expected] += 1
        try:
            predicted = parse_label(prompt)
        except ValueError:
            predicted = None

        if predicted == expected:
            per_label_correct[expected] += 1
        else:
            confusion[expected][predicted] += 1
            if expected is None:
                false_positives.append((prompt, predicted))
            elif predicted is None:
                false_negatives.append((prompt, expected))

    # overall
    total   = len(TEST_PROMPTS)
    correct = sum(per_label_correct.values())

    print("=" * 60)
    print("LABEL DETECTION RESULTS")
    print("=" * 60)
    print(f"\nOverall accuracy: {correct}/{total}  ({100*correct/total:.1f}%)\n")

    print(f"{'Label':<15} {'Correct':>7} {'Total':>7} {'Acc':>7}")
    print("-" * 40)
    for label in list(TEMPLATES.keys()) + [None]:
        c = per_label_correct[label]
        t = per_label_total[label]
        acc = f"{100*c/t:.0f}%" if t > 0 else "N/A"
        name = str(label) if label is not None else "(unknown/None)"
        print(f"{name:<15} {c:>7} {t:>7} {acc:>7}")

    if any(confusion.values()):
        print("\nMisclassifications:")
        for true_label, preds in confusion.items():
            for pred_label, count in preds.items():
                print(f"  true={true_label!r:15s} → predicted={pred_label!r:15s}  ({count}x)")

    if false_negatives:
        print(f"\nFalse negatives (known label not detected):")
        for prompt, expected in false_negatives:
            print(f"  [{expected}] \"{prompt}\"")

    if false_positives:
        print(f"\nFalse positives (unknown prompt matched a label):")
        for prompt, predicted in false_positives:
            print(f"  predicted={predicted!r} for \"{prompt}\"")

    return correct / total


# ---------------------------------------------------------------------------
# IK quality evaluation: per-shape rejection rates + min|det J| stats
# ---------------------------------------------------------------------------
def run_ik_evaluation(n_samples=50):
    print("\n" + "=" * 60)
    print("IK QUALITY EVALUATION")
    print("=" * 60)

    rng = np.random.default_rng(42)
    PARAM_BOUNDS = {"scale": (0.6, 1.5), "rot_deg": (-45.0, 45.0),
                    "dx_mm": (-30.0, 30.0), "dy_mm": (-30.0, 30.0)}
    MIN_DET_THRESHOLD = 1e-4
    MAX_JOINT_JUMP    = 0.3
    DS_MM = 2.0

    print(f"\n{'Shape':<15} {'OK':>5} {'IK fail':>8} {'Sing':>6} {'Flip':>6} {'mindet mean':>12} {'mindet std':>11} {'mindet min':>11}")
    print("-" * 80)

    for label, template_fn in TEMPLATES.items():
        counts = {"ok": 0, "ik_failed": 0, "singularity": 0, "elbow_flip": 0}
        min_dets = []

        for _ in range(n_samples):
            params = {k: rng.uniform(*v) for k, v in PARAM_BOUNDS.items()}
            prog = template_fn()
            x_mm, y_mm, pen = sample_program(prog, ds_mm=DS_MM)
            x_mm, y_mm = apply_params(x_mm, y_mm, **params)
            x_m, y_m = x_mm / 1000.0, y_mm / 1000.0

            thetaAll, min_det, success = solve_trajectory(x_m, y_m, pen)
            if not success:
                counts["ik_failed"] += 1
                continue
            if min_det < MIN_DET_THRESHOLD:
                counts["singularity"] += 1
                continue
            if np.max(np.abs(np.diff(thetaAll, axis=1))) > MAX_JOINT_JUMP:
                counts["elbow_flip"] += 1
                continue
            counts["ok"] += 1
            min_dets.append(min_det)

        md = np.array(min_dets) if min_dets else np.array([0.0])
        print(f"{label:<15} {counts['ok']:>5} {counts['ik_failed']:>8} "
              f"{counts['singularity']:>6} {counts['elbow_flip']:>6} "
              f"{md.mean():>12.5f} {md.std():>11.5f} {md.min():>11.5f}")

    print()


# ---------------------------------------------------------------------------
# Embedding cosine similarity matrix
# ---------------------------------------------------------------------------
def run_embedding_similarity():
    model_path = os.path.join(os.path.dirname(__file__), "model.pt")
    if not os.path.exists(model_path):
        print("\n[skipping embedding similarity — model.pt not found]")
        return

    checkpoint = torch.load(model_path, weights_only=False)
    model = LabelToParams(n_labels=len(LABEL_TO_IDX))
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    labels = list(LABEL_TO_IDX.keys())
    with torch.no_grad():
        embs = model.emb.weight  # (n_labels, emb_dim)
        normed = F.normalize(embs, dim=1)
        sim = (normed @ normed.T).numpy()

    print("\n" + "=" * 60)
    print("EMBEDDING COSINE SIMILARITY MATRIX")
    print("(1.0 = identical direction, 0.0 = orthogonal)")
    print("=" * 60)

    col_w = 12
    header = f"{'':15}" + "".join(f"{l:>{col_w}}" for l in labels)
    print(header)
    print("-" * len(header))
    for i, row_label in enumerate(labels):
        row = f"{row_label:<15}" + "".join(f"{sim[i,j]:>{col_w}.3f}" for j in range(len(labels)))
        print(row)
    print()


# ---------------------------------------------------------------------------
# Per-parameter MSE on test set (requires data/test and model.pt)
# ---------------------------------------------------------------------------
def run_param_mse():
    data_dir = os.path.join(os.path.dirname(__file__), "data", "test")
    model_path = os.path.join(os.path.dirname(__file__), "model.pt")
    if not os.path.exists(data_dir):
        print("\n[skipping per-param MSE — data/test not found, run generate_data.py first]")
        return
    if not os.path.exists(model_path):
        print("\n[skipping per-param MSE — model.pt not found, run train.py first]")
        return

    from train import LabelToParams, LABEL_TO_IDX, PARAM_BOUNDS, PARAM_KEYS, normalize_target

    checkpoint = torch.load(model_path, weights_only=False)
    model = LabelToParams(n_labels=len(LABEL_TO_IDX))
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    files = sorted(glob.glob(os.path.join(data_dir, "*.json")))
    per_label_errors = defaultdict(lambda: np.zeros(4))
    per_label_count  = defaultdict(int)

    with torch.no_grad():
        for f in files:
            with open(f) as fh:
                meta = json.load(fh)
            label = meta["label"]
            idx = torch.tensor([LABEL_TO_IDX[label]])
            pred = model(idx).squeeze(0).numpy()
            true = normalize_target(meta).numpy()
            per_label_errors[label] += (pred - true) ** 2
            per_label_count[label] += 1

    PARAM_KEYS = ["scale", "rot_deg", "dx_mm", "dy_mm"]
    print("\n" + "=" * 60)
    print("PER-LABEL PER-PARAMETER MSE ON TEST SET")
    print("=" * 60)
    header = f"{'Label':<15}" + "".join(f"{k:>12}" for k in PARAM_KEYS) + f"{'mean':>12}"
    print(header)
    print("-" * len(header))
    for label in TEMPLATES.keys():
        n = per_label_count[label]
        if n == 0:
            continue
        mses = per_label_errors[label] / n
        print(f"{label:<15}" + "".join(f"{v:>12.5f}" for v in mses) + f"{mses.mean():>12.5f}")
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    detection_acc = run_label_detection()
    run_ik_evaluation(n_samples=50)
    run_embedding_similarity()
    run_param_mse()

    print("=" * 60)
    print(f"Summary: label detection accuracy = {100*detection_acc:.1f}%")
    print("=" * 60)
