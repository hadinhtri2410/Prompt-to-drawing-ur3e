# Checkpoint 2: Prompt-to-Drawing on a UR3e

---

## Slide 1: Updated Problem Statement & Goal

**Problem:** Physically drawing arbitrary shapes with a robotic arm requires manually crafting joint trajectories — a tedious, expert-only process.

**Goal:** Build an end-to-end pipeline where a user types a natural-language prompt (e.g., `"draw a plane"`) and a UR3e arm draws the corresponding shape on a whiteboard, fully autonomously.

**Pipeline overview:**
```
Prompt → label match → ML param prediction → 2D waypoints → IK → CSV → UR3e
```

**Changes since CP1:**
- CP1 focused on the `csv_controller` (ros2-control plugin) and manual CSV creation
- CP2 adds the full upstream pipeline: drawing primitives, IK solver, dataset generation, and a trained `LabelToParams` model that predicts placement parameters from a shape label
- The system can now go from a text prompt to robot motion without any manual steps

---

## Slide 2: Updated Methodology & Progress

**Technical approach:**

1. **Templates** (`templates.py`) — Shape libraries defined as sequences of `pen_up`, `pen_down`, `line_to`, `circle` commands in 2D millimeter space
2. **Sampling** (`primitives.py`) — Templates are densely sampled into evenly-spaced 2D waypoints at 2 mm resolution
3. **Placement params** — Four parameters (`scale`, `rot_deg`, `dx_mm`, `dy_mm`) transform the 2D shape to fit the reachable workspace
4. **IK solver** (`ik_solver.py`) — Converts 2D waypoints to 6-DOF joint trajectories using iterative Newton-Raphson IK in body frame
5. **Data generation** (`generate_data.py`) — Random params are sampled; trajectories that fail IK, hit singularities, or cause elbow flips are rejected
6. **ML model** (`train.py`) — A small embedding network (`LabelToParams`) learns which params tend to produce valid, well-placed drawings for each shape label
7. **Entry point** (`run_from_prompt.py`) — Parses prompt, runs trained model, executes full pipeline, and launches `csv_controller` on the robot

**Progress since CP1:**

| Component | CP1 | CP2 |
|---|---|---|
| csv_controller (ros2-control) | Done | Done |
| Drawing primitives + IK | Not started | Done |
| Dataset generation | Not started | Done |
| ML model (LabelToParams) | Not started | Done |
| End-to-end prompt → robot | Not started | Done (sim verified) |

---

## Slide 3: Code Snippet 1 — Embedding-Based Neural Network (`LabelToParams`)

```python
PARAM_BOUNDS = {"scale": (0.6, 1.5), "rot_deg": (-45, 45),
                "dx_mm": (-30, 30),  "dy_mm":   (-30, 30)}

class LabelToParams(nn.Module):
    def __init__(self, n_labels, emb=16):
        super().__init__()
        self.emb = nn.Embedding(n_labels, emb)
        self.net = nn.Sequential(
            nn.Linear(emb, 32), nn.ReLU(),
            nn.Linear(32, 4),   nn.Sigmoid()
        )
    def forward(self, label_idx):
        z = self.emb(label_idx)
        return self.net(z)

def normalize_target(meta):
    vals = []
    for key in ["scale", "rot_deg", "dx_mm", "dy_mm"]:
        lo, hi = PARAM_BOUNDS[key]
        vals.append((meta[key] - lo) / (hi - lo))
    return torch.tensor(vals, dtype=torch.float32)
```

---

## Slide 4: Explanation of Snippet 1

**What it does:**

- `nn.Embedding(n_labels, 16)` maps each discrete shape label (e.g. `"plane"` → index 0) to a learned 16-dimensional dense vector. This lets the model treat labels as points in a continuous space rather than unordered integers.
- The embedding feeds into a two-layer MLP: `Linear(16→32) → ReLU → Linear(32→4) → Sigmoid`
- The final `Sigmoid` clamps outputs to (0, 1), which maps back to physical units via `PARAM_BOUNDS` — e.g. output 0.8 for `scale` → `0.8 × (1.5 − 0.6) + 0.6 = 1.32`
- `normalize_target` does the inverse: converts ground-truth params into [0, 1] so MSE loss operates in a consistent, bounded space

**Why this is the core AI component:**

The entire ML problem reduces to: *given a shape label, predict the 4 placement parameters that will produce a valid, well-placed drawing on the whiteboard*. The embedding layer is what makes this more than a lookup table — it gives the model a learnable geometry over the label space, so similar shapes can share structure in the embedding space.

---

## Slide 5: Code Snippet 2 — Training Loop with Adam and MSE

```python
def train(data_root="data", epochs=200, lr=1e-3, batch_size=64):
    train_dl = DataLoader(ParamDataset(data_root+"/train", LABEL_TO_IDX),
                          batch_size=batch_size, shuffle=True)
    val_dl   = DataLoader(ParamDataset(data_root+"/val",   LABEL_TO_IDX),
                          batch_size=batch_size)
    model     = LabelToParams(n_labels=len(LABEL_TO_IDX))
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    best_val  = float("inf")

    for epoch in range(epochs):
        model.train()
        for x, y in train_dl:
            pred = model(x)
            loss = F.mse_loss(pred, y)
            optimizer.zero_grad(); loss.backward(); optimizer.step()

        model.eval()
        with torch.no_grad():
            val_loss = sum(F.mse_loss(model(x), y).item() * x.size(0)
                           for x, y in val_dl) / len(val_dl.dataset)
        if val_loss < best_val:
            best_val = val_loss
            torch.save({"model_state": model.state_dict(), ...}, "model.pt")
```

---

## Slide 6: Explanation of Snippet 2

**What it does:**

- Trains `LabelToParams` using **Adam** (adaptive gradient descent) with MSE loss over 200 epochs
- Each batch: forward pass → MSE loss between predicted and ground-truth normalized params → `loss.backward()` computes gradients → `optimizer.step()` updates all weights including the embedding vectors
- Validation loss is computed every epoch without gradient tracking (`torch.no_grad()`) — the best checkpoint by val loss is saved to `model.pt`
- **Why MSE, not cross-entropy:** the 4 outputs are continuous regression targets (real-valued params normalized to [0,1]), not class probabilities. MSE penalizes how far the predicted value is from the true value, which is the right signal for regression.
- **Why Adam, not plain SGD:** Adam adapts the learning rate per parameter, which helps when the embedding gradients for rare labels are sparse — those rows only receive gradient updates when that label appears in the batch

**Why this is a key AI component:**

This loop is where all learning happens. The embedding table, the hidden layer weights, and the output layer are all jointly optimized to minimize the gap between predicted and physically-validated placement params.

---

## Slide 7: New Preliminary Result

**Result: End-to-end pipeline verified in simulation**

`python run_from_prompt.py "draw a plane"`

```
Prompt: "draw a plane"
Matched template: plane
Params: {'scale': 0.95, 'rot_deg': 3.2, 'dx_mm': -4.1, 'dy_mm': 1.7}
Running IK for 'plane' (312 waypoints) ...
  IK done. min |det(Jb)| = 0.003241
  Saved: csv_files/plane.csv
```

- IK converged on all 312 waypoints for the plane template
- `min |det(Jb)| = 0.003` — well above singularity threshold
- Generated CSV replayed in RViz via `msee22_description move_robot.launch.py` and joint trajectory visually matches the plane outline
- Dataset generation: 300 train / 50 val / 50 test samples per label, ~18% rejection rate (IK failures + singularities + elbow flips)

> **[INSERT: RViz screenshot or matplotlib plot of the 2D waypoints overlaid with the drawn shape]**

---

## Slide 8: Result Analysis & Next Steps

**What this result means:**

- The full pipeline is working end-to-end in simulation — a text prompt produces a physically valid joint trajectory without any manual steps
- The 18% rejection rate during data generation confirms that not all param combinations are feasible, validating the need for the ML model to predict "good" params rather than using random or fixed defaults
- The `min |det(Jb)|` singularity metric gives a quantitative measure of trajectory quality beyond just IK convergence

**What still needs validation:**

- The `LabelToParams` model has not yet been trained on real data — currently `run_from_prompt.py` falls back to default params if no `model.pt` exists
- Real robot execution has not been tested yet (only RViz simulation)

**Immediate next steps:**

1. Run `generate_data.py` to completion and train the `LabelToParams` model — verify val loss converges
2. Execute the generated CSV on the real UR3e and photograph the result on the whiteboard
3. Add at least 2 more shape templates (e.g., house, star) to make the label space more interesting
4. Evaluate: does the model predict meaningfully different params per shape, or does it collapse to similar values?
