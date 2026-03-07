import torch 
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import os, json, glob
import numpy as np
from templates import TEMPLATES

PARAM_BOUNDS = {"scale": (0.6, 1.5), "rot_deg": (-45, 45), "dx_mm": (-30, 30), "dy_mm": (-30, 30)}

class ParamDataset(torch.utils.data.Dataset): 
    def __init__(self, data_dir, label_to_idx):
        self.files = sorted(glob.glob(os.path.join(data_dir, "*.json")))
        self.label_to_idx = label_to_idx
    def __len__(self): 
        return len(self.files)
    def __getitem__(self, i):
        with open(self.files[i]) as f:
            meta = json.load(f)
        x = torch.tensor(self.label_to_idx[meta["label"]], dtype=torch.long)
        y = normalize_target(meta)
        return x, y

class LabelToParams(nn.Module):
    def __init__(self, n_labels, emb=16):
        super().__init__()
        self.emb = nn.Embedding(n_labels, emb)
        self.net = nn.Sequential(
            nn.Linear(emb, 32), nn.ReLU(),
            nn.Linear(32, 4), nn.Sigmoid()
        )
    def forward(self, label_idx):
        z = self.emb(label_idx)
        return self.net(z)

PARAM_KEYS = ["scale", "rot_deg", "dx_mm", "dy_mm"]
LABEL_TO_IDX = {label: i for i, label in enumerate(TEMPLATES.keys())}


def normalize_target(meta):
    vals = []
    for key in PARAM_KEYS:
        lo, hi = PARAM_BOUNDS[key]
        vals.append((meta[key] - lo) / (hi - lo))
    return torch.tensor(vals, dtype=torch.float32)


def denormalize(t):
    out = {}
    for i, key in enumerate(PARAM_KEYS):
        lo, hi = PARAM_BOUNDS[key]
        out[key] = t[i].item() * (hi - lo) + lo
    return out


def train(data_root="data", epochs=200, lr=1e-3, batch_size=64):
    train_ds = ParamDataset(os.path.join(data_root, "train"), LABEL_TO_IDX)
    val_ds = ParamDataset(os.path.join(data_root, "val"), LABEL_TO_IDX)

    train_dl = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_dl = DataLoader(val_ds, batch_size=batch_size)

    model = LabelToParams(n_labels=len(LABEL_TO_IDX))
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    best_val_loss = float("inf")

    for epoch in range(epochs):
        # train
        model.train()
        train_loss = 0.0
        for x, y in train_dl:
            pred = model(x)
            loss = F.mse_loss(pred, y)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * x.size(0)
        train_loss /= len(train_ds)

        # val
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for x, y in val_dl:
                pred = model(x)
                val_loss += F.mse_loss(pred, y).item() * x.size(0)
        val_loss /= len(val_ds)

        if (epoch + 1) % 20 == 0 or epoch == 0:
            print(f"Epoch {epoch+1:3d}  train_loss={train_loss:.6f}  val_loss={val_loss:.6f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save({
                "model_state": model.state_dict(),
                "label_to_idx": LABEL_TO_IDX,
                "param_bounds": PARAM_BOUNDS,
            }, "model.pt")

    print(f"\nBest val loss: {best_val_loss:.6f}")
    print("Saved model.pt")


if __name__ == "__main__":
    train()
