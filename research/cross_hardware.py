#!/usr/bin/env python3
"""Reproduces Tables 3-6 from data/real_10subj.csv (the wrist-mounted,
N=10 collection) and data/lyra_multisubject.csv (the original N=3
breadboard collection). Run from the repo root:

  python3 research/cross_hardware.py

This is the second half of the paper's reproduction story -- lyra_ml.py
covers Tables 1-2 on its own, this script covers everything that needs
both datasets or the standalone classify_fast/classify_hybrid firmware
port.
"""
import sys
import os
import math
sys.path.insert(0, os.path.dirname(__file__))

from lyra_ml import (load, loso, build_tree, tree_predict, clf_tree, clf_hybrid,
                      summarize, FEATURES, GYRO_FLOOR, FLICK_GYRO, RATIO_CUT)
from firmware_port import classify_fast, classify_hybrid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def wilson(hit, n, z=1.96):
    if n == 0:
        return (None, None)
    p = hit / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / denom
    return (round(100 * (center - half), 1), round(100 * (center + half), 1))


def gesture_acc(rows, truth_key, classify):
    out = {}
    for g in ["PINCH", "FLICK", "TWIST"]:
        sub = [r for r in rows if r[truth_key] == g]
        if not sub:
            continue
        hit = sum(1 for r in sub if classify(r) == g)
        lo, hi = wilson(hit, len(sub))
        out[g] = (round(100 * hit / len(sub), 1), lo, hi, hit, len(sub))
    return out


def print_table(title, rows):
    print(f"\n=== {title} ===")
    for label, acc in rows:
        bits = []
        for g, v in acc.items():
            if v[1] is None:
                bits.append(f"{g} {v[0]}% ({v[3]}/{v[4]})")
            else:
                bits.append(f"{g} {v[0]}% [{v[1]}-{v[2]}] ({v[3]}/{v[4]})")
        print(f"  {label:32s}  {'  '.join(bits)}")


# --- load both datasets ------------------------------------------------
wrist_rows, _ = load(os.path.join(ROOT, "data", "real_10subj.csv"))
bread_rows, _ = load(os.path.join(ROOT, "data", "lyra_multisubject.csv"))
subjects = sorted(set(r["subject"] for r in bread_rows))
full = [s for s in subjects
        if all(any(r["subject"] == s and r["truth"] == g for r in bread_rows)
               for g in ["FLICK", "TWIST", "PINCH"])]
bread_lrows = [r for r in bread_rows if r["subject"] in full]


def to_classify_feat(r):
    return {
        "peakGyro": r["peakGyro"], "ratio": r["ratio"],
        "durationCount": r["durationCount"], "gySigned": r["gySigned"],
        "gzSigned": r["gzSigned"], "peakAccel": r["peakAccel"],
        "accelMean": r["accelMean"], "accelStd": r["accelStd"],
        "spikeCount": r["spikeCount"],
    }


def deployed_hybrid_loso(train_rows, held_subjects):
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
    return loso(train_rows, held_subjects, clf_hybrid_deployed)


# --- Table 3: rules-only and hybrid, both rigs -------------------------
rules_breadboard = gesture_acc(bread_lrows, "truth", lambda r: r["device"])
rules_wrist = gesture_acc(wrist_rows, "truth", lambda r: classify_fast(to_classify_feat(r)))

hybrid_bread_pairs = deployed_hybrid_loso(bread_lrows, full)
hybrid_bread = {}
for g in ["PINCH", "FLICK", "TWIST"]:
    sub = [(t, p) for (t, p) in hybrid_bread_pairs if t == g]
    hit = sum(1 for t, p in sub if p == g)
    hybrid_bread[g] = (round(100 * hit / len(sub), 1), None, None, hit, len(sub))

hybrid_wrist = gesture_acc(wrist_rows, "truth", lambda r: classify_hybrid(to_classify_feat(r)))

print_table("Table 3 -- Rules-only, breadboard (N=3)", [("Rules, breadboard", rules_breadboard)])
print_table("Table 3 -- Rules-only, wrist (N=10)", [("Rules, wrist", rules_wrist)])
print_table("Table 3 -- Hybrid, breadboard (N=3, LOSO)", [("Hybrid, breadboard", hybrid_bread)])
print_table("Table 3 -- Hybrid, wrist (N=10)", [("Hybrid, wrist", hybrid_wrist)])

# --- Table 4: per-participant hybrid, ten wrist-mounted participants ---
print("\n=== Table 4 -- per-participant hybrid accuracy, wrist (N=10) ===")
for p in sorted(set(r["subject"] for r in wrist_rows)):
    prows = [r for r in wrist_rows if r["subject"] == p]
    acc = gesture_acc(prows, "truth", lambda r: classify_hybrid(to_classify_feat(r)))
    parts = "  ".join(f"{g} {v[0]}% (n={v[4]})" for g, v in acc.items())
    print(f"  {p:4s}  {parts}")

# --- Table 5: within-subject, A/B/C on both rigs, both classifiers -----
# "Hybrid, breadboard" here is NOT LOSO -- it is the same framing as Table 1:
# the tree as actually deployed (fit once on all three breadboard subjects,
# depth 4 / min_leaf 3, the hyperparameters that reproduce the shipped
# 422.05 split) evaluated per person. That matches what "the deployed
# classifier" means physically: one fixed tree flashed to the device, not a
# different tree per held-out fold.
deployed_residual = [r for r in bread_lrows
                      if not (r["peakGyro"] < GYRO_FLOOR
                              or (r["peakGyro"] >= FLICK_GYRO and r["ratio"] >= RATIO_CUT))]
deployed_ys = [r["truth"] for r in deployed_residual]
deployed_node = build_tree(deployed_residual, FEATURES, deployed_ys, 0, 4, min_leaf=3)


def classify_hybrid_deployed_single(r):
    if r["peakGyro"] < GYRO_FLOOR:
        return "NONE"
    if r["peakGyro"] >= FLICK_GYRO and r["ratio"] >= RATIO_CUT:
        return "FLICK"
    return tree_predict(deployed_node, r)

print("\n=== Table 5 -- within-subject, A/B/C, both rigs, both classifiers ===")
for person in ["A", "B", "C"]:
    for gesture in ["TWIST", "PINCH"]:
        bread_sub = [r for r in bread_lrows if r["subject"] == person and r["truth"] == gesture]
        wrist_sub = [r for r in wrist_rows if r["subject"] == person and r["truth"] == gesture]

        rules_b_hit = sum(1 for r in bread_sub if r["device"] == gesture)
        rules_w_hit = sum(1 for r in wrist_sub if classify_fast(to_classify_feat(r)) == gesture)

        hybrid_b_hit = sum(1 for r in bread_sub if classify_hybrid_deployed_single(r) == gesture)
        hybrid_w_hit = sum(1 for r in wrist_sub if classify_hybrid(to_classify_feat(r)) == gesture)

        print(f"  {person} {gesture:6s}  rules_breadboard={rules_b_hit}/{len(bread_sub)}"
              f"  hybrid_breadboard={hybrid_b_hit}/{len(bread_sub)}"
              f"  rules_wrist={rules_w_hit}/{len(wrist_sub)}"
              f"  hybrid_wrist={hybrid_w_hit}/{len(wrist_sub)}")

# --- Table 6: stage-2 tree and hybrid, refit and LOSO'd at N=10 --------

wrist_subjects = sorted(set(r["subject"] for r in wrist_rows))
print("\n=== Table 6 -- refit at N=10, wrist dataset ===")
summarize(loso(wrist_rows, wrist_subjects, clf_tree(3)), "pure tree, refit N=10")
summarize(loso(wrist_rows, wrist_subjects, clf_hybrid(3)), "hybrid, refit N=10")
