#!/usr/bin/env python3
"""
lyra_ml.py — leave-one-subject-out (LOSO) verification harness for the LYRA
multi-subject gesture dataset. Pure Python standard library only (no numpy /
pandas / sklearn), so it runs anywhere.

It exists to make the paper's numbers reproducible from the committed data:

  1. Parses the 12-column CSV (header-driven; skips '#' comments and blank lines),
     and drops EXACT duplicate rows (reporting how many) so pasted-in duplicate
     blocks can't silently inflate a subject.
  2. Table 1 — per-subject, per-gesture accuracy of the device's own on-board call
     (the `classified` column) vs ground truth (`label`). This needs NO model:
     it is exactly the "rule-based generalization, per participant" table.
  3. Table 2 — pooled LOSO comparison of: rules (the `classified` column),
     a CART decision tree, one-vs-rest logistic regression, and the deployed
     hybrid (stage-1 rules for NONE/FLICK + stage-2 tree for twist/pinch).
  4. Emits the chosen stage-2 tree as copy-pasteable C++.

Usage:
  python3 lyra_ml.py data.csv
  python3 lyra_ml.py data.csv --depth 3        # stage-2 tree max depth
  python3 lyra_ml.py data.csv --keep-subject D       # include a subject usually held out

Schema (order-independent; matched by header name):
  subject,label,peakGyro,ratio,durationCount,gySigned,gzSigned,
  peakAccel,accelMean,accelStd,spikeCount,classified
"""

import sys
import math

# The 9 numeric features used for the learned models.
FEATURES = ["peakGyro", "ratio", "durationCount", "gySigned", "gzSigned",
            "peakAccel", "accelMean", "accelStd", "spikeCount"]

GESTURES = ["FLICK", "TWIST", "PINCH", "NONE"]

# Stage-1 rule reference thresholds (from the firmware / README).
GYRO_FLOOR = 220.0      # peakGyro below this -> NONE
FLICK_GYRO = 700.0      # high-gyro boundary for a confident flick
RATIO_CUT  = 1.20       # axis-concentration cutoff (flick = energy in one axis)


# ----------------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------------
def load(path):
    """Return (rows, n_dupes). Each row is a dict. GARBAGE label -> NONE truth."""
    with open(path) as f:
        raw = [ln.rstrip("\n") for ln in f]

    header = None
    rows = []
    seen = set()
    n_dupes = 0
    for ln in raw:
        s = ln.strip()
        if not s or s.startswith("#"):
            continue
        parts = [p.strip() for p in s.split(",")]
        # Detect the header line.
        if header is None and "label" in parts and "subject" in parts:
            header = parts
            continue
        # If there is no header, assume canonical order.
        if header is None:
            header = ["subject", "label", "peakGyro", "ratio", "durationCount",
                      "gySigned", "gzSigned", "peakAccel", "accelMean",
                      "accelStd", "spikeCount", "classified"]
        if len(parts) != len(header):
            # Malformed row (e.g. an old 7-column export) — skip and note.
            continue
        key = s
        if key in seen:
            n_dupes += 1
            continue
        seen.add(key)
        d = dict(zip(header, parts))
        # Normalise truth: GARBAGE -> NONE (a correct rejection).
        d["truth"] = "NONE" if d["label"].upper() == "GARBAGE" else d["label"].upper()
        d["device"] = d.get("classified", "").upper() or "NONE"
        ok = True
        for feat in FEATURES:
            try:
                d[feat] = float(d[feat])
            except (KeyError, ValueError):
                ok = False
                break
        if ok:
            rows.append(d)
    return rows, n_dupes


# ----------------------------------------------------------------------------
# Metrics
# ----------------------------------------------------------------------------
def per_gesture_accuracy(rows, pred_key):
    """Accuracy per real gesture (FLICK/TWIST/PINCH) = correct / total of that class."""
    out = {}
    for g in ["PINCH", "FLICK", "TWIST"]:
        sub = [r for r in rows if r["truth"] == g]
        if not sub:
            out[g] = None
        else:
            hit = sum(1 for r in sub if r[pred_key] == g)
            out[g] = hit / len(sub)
    return out


def false_positive_rate(rows, pred_key):
    """Fraction of GARBAGE (truth NONE) windows misfired as a real gesture."""
    noise = [r for r in rows if r["truth"] == "NONE"]
    if not noise:
        return None
    fp = sum(1 for r in noise if r[pred_key] != "NONE")
    return fp / len(noise)


def fmt_pct(x):
    return " n/a " if x is None else f"{100*x:4.0f}%"


# ----------------------------------------------------------------------------
# CART decision tree (Gini), stdlib
# ----------------------------------------------------------------------------
class Node:
    __slots__ = ("feat", "thr", "left", "right", "label")

    def __init__(self):
        self.feat = None; self.thr = None
        self.left = None; self.right = None
        self.label = None


def _gini(labels):
    n = len(labels)
    if n == 0:
        return 0.0
    counts = {}
    for l in labels:
        counts[l] = counts.get(l, 0) + 1
    return 1.0 - sum((c / n) ** 2 for c in counts.values())


def _majority(labels):
    counts = {}
    for l in labels:
        counts[l] = counts.get(l, 0) + 1
    return max(counts, key=counts.get)


def build_tree(rows, feats, ys, depth, max_depth, min_leaf=4):
    node = Node()
    if depth >= max_depth or len(set(ys)) <= 1 or len(ys) < 2 * min_leaf:
        node.label = _majority(ys)
        return node
    base = _gini(ys)
    best = None  # (gain, feat, thr, li, ri)
    n = len(rows)
    for f in feats:
        vals = sorted(set(r[f] for r in rows))
        for i in range(len(vals) - 1):
            thr = (vals[i] + vals[i + 1]) / 2.0
            li = [j for j in range(n) if rows[j][f] <= thr]
            ri = [j for j in range(n) if rows[j][f] > thr]
            if len(li) < min_leaf or len(ri) < min_leaf:
                continue
            gl = _gini([ys[j] for j in li]); gr = _gini([ys[j] for j in ri])
            gain = base - (len(li) * gl + len(ri) * gr) / n
            if best is None or gain > best[0]:
                best = (gain, f, thr, li, ri)
    if best is None or best[0] <= 1e-9:
        node.label = _majority(ys)
        return node
    _, f, thr, li, ri = best
    node.feat = f; node.thr = thr
    node.left = build_tree([rows[j] for j in li], feats, [ys[j] for j in li],
                           depth + 1, max_depth, min_leaf)
    node.right = build_tree([rows[j] for j in ri], feats, [ys[j] for j in ri],
                            depth + 1, max_depth, min_leaf)
    return node


def tree_predict(node, row):
    while node.label is None:
        node = node.left if row[node.feat] <= node.thr else node.right
    return node.label


# ----------------------------------------------------------------------------
# One-vs-rest logistic regression (standardized features, gradient descent)
# ----------------------------------------------------------------------------
def _standardize(train, test, feats):
    mu = {f: sum(r[f] for r in train) / len(train) for f in feats}
    sd = {}
    for f in feats:
        var = sum((r[f] - mu[f]) ** 2 for r in train) / max(1, len(train))
        sd[f] = math.sqrt(var) or 1.0
    def z(r):
        return [(r[f] - mu[f]) / sd[f] for f in feats]
    return [z(r) for r in train], [z(r) for r in test]


def logreg_train(X, y_bin, iters=300, lr=0.1):
    n = len(X); d = len(X[0]) if X else 0
    w = [0.0] * d; b = 0.0
    for _ in range(iters):
        gw = [0.0] * d; gb = 0.0
        for i in range(n):
            z = b + sum(w[k] * X[i][k] for k in range(d))
            p = 1.0 / (1.0 + math.exp(-max(-30, min(30, z))))
            err = p - y_bin[i]
            for k in range(d):
                gw[k] += err * X[i][k]
            gb += err
        for k in range(d):
            w[k] -= lr * gw[k] / n
        b -= lr * gb / n
    return w, b


def logreg_predict(models, x):
    best = None; blabel = "NONE"
    for label, (w, b) in models.items():
        z = b + sum(w[k] * x[k] for k in range(len(x)))
        p = 1.0 / (1.0 + math.exp(-max(-30, min(30, z))))
        if best is None or p > best:
            best = p; blabel = label
    return blabel


# ----------------------------------------------------------------------------
# LOSO evaluation
# ----------------------------------------------------------------------------
def loso(rows, subjects, classifier):
    """Return combined predictions list of (truth, pred) across all held-out folds."""
    out = []
    for held in subjects:
        train = [r for r in rows if r["subject"] != held]
        test = [r for r in rows if r["subject"] == held]
        preds = classifier(train, test)
        for r, p in zip(test, preds):
            out.append((r["truth"], p))
    return out


def clf_tree(depth):
    def run(train, test):
        ys = [r["truth"] for r in train]
        node = build_tree(train, FEATURES, ys, 0, depth)
        return [tree_predict(node, r) for r in test]
    return run


def clf_hybrid(depth):
    """Stage-1 rules for NONE/FLICK; stage-2 CART for the twist/pinch residual."""
    def run(train, test):
        residual = [r for r in train
                    if not (r["peakGyro"] < GYRO_FLOOR
                            or (r["peakGyro"] >= FLICK_GYRO and r["ratio"] >= RATIO_CUT))]
        ys = [r["truth"] for r in residual]
        node = build_tree(residual, FEATURES, ys, 0, depth) if len(set(ys)) > 1 else None
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
    return run


def clf_logreg(train, test):
    labels = sorted(set(r["truth"] for r in train))
    Xtr, Xte = _standardize(train, test, FEATURES)
    models = {}
    for lab in labels:
        yb = [1 if r["truth"] == lab else 0 for r in train]
        models[lab] = logreg_train(Xtr, yb)
    return [logreg_predict(models, x) for x in Xte]


def summarize(pairs, name):
    class Row: pass
    def acc(g):
        sub = [(t, p) for (t, p) in pairs if t == g]
        return None if not sub else sum(1 for t, p in sub if p == g) / len(sub)
    noise = [(t, p) for (t, p) in pairs if t == "NONE"]
    fp = None if not noise else sum(1 for t, p in noise if p != "NONE") / len(noise)
    print(f"  {name:16s}  Pinch {fmt_pct(acc('PINCH'))}  Flick {fmt_pct(acc('FLICK'))}"
          f"  Twist {fmt_pct(acc('TWIST'))}  FP {fmt_pct(fp)}")


# ----------------------------------------------------------------------------
# C++ emit
# ----------------------------------------------------------------------------
def emit_cpp(node, indent=1):
    pad = "  " * indent
    if node.label is not None:
        return f'{pad}return "{node.label}";\n'
    s = f"{pad}if ({node.feat} <= {node.thr:.4f}f) {{\n"
    s += emit_cpp(node.left, indent + 1)
    s += f"{pad}}} else {{\n"
    s += emit_cpp(node.right, indent + 1)
    s += f"{pad}}}\n"
    return s


# ----------------------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    path = sys.argv[1]
    depth = 3
    keep = set()
    if "--depth" in sys.argv:
        depth = int(sys.argv[sys.argv.index("--depth") + 1])
    if "--keep-subject" in sys.argv:
        keep.add(sys.argv[sys.argv.index("--keep-subject") + 1])

    rows, n_dupes = load(path)
    subjects = sorted(set(r["subject"] for r in rows))

    print(f"\nLoaded {len(rows)} unique rows  (dropped {n_dupes} exact duplicates)")
    print(f"Subjects: {', '.join(subjects)}\n")

    print("Per-subject sample counts (truth):")
    for s in subjects:
        srows = [r for r in rows if r["subject"] == s]
        counts = {g: sum(1 for r in srows if r["truth"] == g) for g in GESTURES}
        note = ""
        real = sum(counts[g] for g in ["FLICK", "TWIST", "PINCH"])
        if counts["FLICK"] and not (counts["TWIST"] or counts["PINCH"]):
            note = "  <-- FLICK-only; cannot serve as a twist/pinch LOSO fold"
        print(f"  {s:8s}  FLICK {counts['FLICK']:3d}  TWIST {counts['TWIST']:3d}"
              f"  PINCH {counts['PINCH']:3d}  NONE {counts['NONE']:3d}{note}")

    print("\n=== Table 1: rule-based generalization, per participant "
          "(device `classified` vs `label`) ===")
    print("  subject           Pinch   Flick   Twist")
    for s in subjects:
        srows = [r for r in rows if r["subject"] == s]
        a = per_gesture_accuracy(srows, "device")
        print(f"  {s:16s}  {fmt_pct(a['PINCH'])}  {fmt_pct(a['FLICK'])}  {fmt_pct(a['TWIST'])}")

    # Full LOSO subjects = those with all three real gestures (unless kept).
    full = [s for s in subjects
            if all(any(r["subject"] == s and r["truth"] == g for r in rows)
                   for g in ["FLICK", "TWIST", "PINCH"]) or s in keep]
    excluded = [s for s in subjects if s not in full]
    lrows = [r for r in rows if r["subject"] in full]
    if excluded:
        print(f"\n(LOSO uses N={len(full)} complete subjects: {', '.join(full)}. "
              f"Excluded (incomplete): {', '.join(excluded)}.)")

    print("\n=== Table 2: pooled LOSO across complete subjects ===")
    # Rules = the device `classified` column, pooled over the LOSO subjects.
    summarize([(r["truth"], r["device"]) for r in lrows], "Rules (device)")
    summarize(loso(lrows, full, clf_tree(depth)),   f"Decision tree d{depth}")
    summarize(loso(lrows, full, clf_logreg),        "Logistic reg.")
    summarize(loso(lrows, full, clf_hybrid(depth)), "Hybrid (deployed)")

    # Emit a stage-2 tree trained on all complete-subject twist/pinch residual.
    residual = [r for r in lrows
                if not (r["peakGyro"] < GYRO_FLOOR
                        or (r["peakGyro"] >= FLICK_GYRO and r["ratio"] >= RATIO_CUT))]
    ys = [r["truth"] for r in residual]
    if len(set(ys)) > 1:
        node = build_tree(residual, FEATURES, ys, 0, depth)
        print("\n=== Stage-2 tree as C++ (twist/pinch residual) ===")
        print("const char* stage2() {")
        print(emit_cpp(node), end="")
        print("}")


if __name__ == "__main__":
    main()
