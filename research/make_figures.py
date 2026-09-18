#!/usr/bin/env python3
"""Regenerate Figures 1-3 from data/lyra_multisubject.csv, using the same
LOSO pipeline as lyra_ml.py. Run from the repo root:

  python3 research/make_figures.py

Writes lyra_fig1_accelstd_separation.png, lyra_fig2_tree_overfitting.png,
and lyra_fig3_before_after.png into figures/.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from lyra_ml import (load, loso, clf_tree, build_tree, tree_predict,
                      GYRO_FLOOR, FLICK_GYRO, RATIO_CUT, FEATURES)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data", "lyra_multisubject.csv")
OUT = os.path.join(ROOT, "figures")
os.makedirs(OUT, exist_ok=True)

rows, _ = load(DATA)
subjects = sorted(set(r["subject"] for r in rows))
full = [s for s in subjects
        if all(any(r["subject"] == s and r["truth"] == g for r in rows)
               for g in ["FLICK", "TWIST", "PINCH"])]
lrows = [r for r in rows if r["subject"] in full]

# accelStd distribution, pinch vs incidental motion (NONE-truth) -- Figure 1
pinch_std = [r["accelStd"] for r in lrows if r["truth"] == "PINCH"]
none_std = [r["accelStd"] for r in lrows if r["truth"] == "NONE"]

fig, ax = plt.subplots(figsize=(6.4, 4.2))
bins = [i / 20 for i in range(0, 18)]
ax.hist(none_std, bins=bins, label=f"Incidental Motion (n={len(none_std)})")
ax.hist(pinch_std, bins=bins, label=f"Pinch (n={len(pinch_std)})", alpha=0.75)
ax.set_title("accelStd Distribution by Gesture")
ax.set_xlabel("accelStd (Std Dev of Acceleration Magnitude, g)")
ax.set_ylabel("Count")
ax.grid(True, alpha=0.3)
ax.legend()
fig.savefig(f"{OUT}/lyra_fig1_accelstd_separation.png", dpi=150)
print("fig1: pinch n=", len(pinch_std), "none n=", len(none_std))

# Figure 2: pure-tree accuracy vs depth, per gesture, LOSO on the 3 breadboard subjects
depths = list(range(1, 9))
acc_by_gesture = {"FLICK": [], "TWIST": [], "PINCH": []}
for d in depths:
    pairs = loso(lrows, full, clf_tree(d))
    for g in acc_by_gesture:
        sub = [(t, p) for (t, p) in pairs if t == g]
        acc_by_gesture[g].append(100 * sum(1 for t, p in sub if p == g) / len(sub))

fig, ax = plt.subplots(figsize=(6.8, 3.8))
for g in ["FLICK", "TWIST", "PINCH"]:
    ax.plot(depths, acc_by_gesture[g], marker="o", label=g.capitalize())
ax.set_title("Flick Accuracy vs. Tree Depth")
ax.set_xlabel("Pure-Tree Max Depth")
ax.set_ylabel("LOSO Accuracy (%)")
ax.set_ylim(0, 105)
ax.legend(frameon=False)
fig.savefig(f"{OUT}/lyra_fig2_tree_overfitting.png", dpi=150)
print("fig2: flick accuracy by depth ->", dict(zip(depths, [round(v, 1) for v in acc_by_gesture["FLICK"]])))

# and Figure 3 -- rules vs deployed hybrid, same numbers as Table 2
def clf_hybrid_deployed(train, test):
    residual = [r for r in train
                if not (r["peakGyro"] < GYRO_FLOOR
                        or (r["peakGyro"] >= FLICK_GYRO and r["ratio"] >= RATIO_CUT))]
    ys = [r["truth"] for r in residual]
    node = build_tree(residual, FEATURES, ys, 0, 4, min_leaf=3) if len(set(ys)) > 1 else None
    preds = []
    for r in test:
        if r["peakGyro"] < GYRO_FLOOR:
            preds.append("NONE")
        elif r["peakGyro"] >= FLICK_GYRO and r["ratio"] >= RATIO_CUT:
            preds.append("FLICK")
        elif node is not None:
            preds.append(tree_predict(node, r))
        else:
            preds.append("TWIST")
    return preds

rules_pairs = [(r["truth"], r["device"]) for r in lrows]
hybrid_pairs = loso(lrows, full, clf_hybrid_deployed)

def accs(pairs):
    out = {}
    for g in ["PINCH", "FLICK", "TWIST"]:
        sub = [(t, p) for (t, p) in pairs if t == g]
        out[g] = 100 * sum(1 for t, p in sub if p == g) / len(sub)
    return out

rules_acc = accs(rules_pairs)
hybrid_acc = accs(hybrid_pairs)
print("fig3: rules ->", rules_acc)
print("fig3: hybrid ->", hybrid_acc)

gestures = ["Pinch", "Flick", "Twist"]
rules_vals = [rules_acc[g.upper()] for g in gestures]
hybrid_vals = [hybrid_acc[g.upper()] for g in gestures]

fig, ax = plt.subplots(figsize=(6.0, 4.3))
x = range(len(gestures))
w = 0.35
ax.bar([i - w / 2 for i in x], rules_vals, width=w, label="Rules Only")
ax.bar([i + w / 2 for i in x], hybrid_vals, width=w, label="Deployed Hybrid")
ax.set_title("Accuracy: Rules vs. Hybrid")
ax.set_xticks(list(x))
ax.set_xticklabels(gestures)
ax.set_ylabel("LOSO Accuracy (%)")
ax.set_ylim(0, 105)
ax.legend()
fig.savefig(f"{OUT}/lyra_fig3_before_after.png", dpi=150)
print("done")
