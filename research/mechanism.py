#!/usr/bin/env python3
"""
mechanism.py — why the hybrid survives the breadboard -> wrist mounting change
and the fixed rules do not. Counterfactual experiments behind the paper's
mechanism analysis (the component swap, critical gain and gain sweep).

The v2 paper observed the effect (rules collapse, hybrid holds) and guessed at a
cause ("the tree captures shape, not magnitude"). This script tests causes
directly, on the committed data only, with no retraining on the wrist data:

  E1  Component swap (2x2 factorial). The rules' twist/pinch separator and pinch
      gate are swapped one at a time for the deployed tree's, and the reverse:
      the deployed tree's are swapped for the rules'. Paired exact McNemar tests
      and a subject-cluster bootstrap CI on the wrist data.
  E2  Same structure, fitted thresholds. The hand-set rule cascade is kept
      exactly, and only its two numbers (700 dps, 0.40 g) are re-chosen by a
      one-split Gini fit on the breadboard data.
  E3  Critical gain. For every decision boundary, the multiplicative change in
      its feature at which the protected class's median crosses it (k*), vs
      the gain actually observed between rigs.
  E4  Gain sweep (dose-response). Breadboard features are rescaled
      synthetically (accelerometer deviation by ka, gyro amplitude by kg) and
      both deployed classifiers are re-scored. Also replays the observed gain.
  E5  Learned trees in general. 300 bootstrap stage-2 trees plus feature-subset
      trees (amplitude-only, no-amplitude, ...), all fit on breadboard and
      scored on wrist, to test the v2 "shape not magnitude" explanation.
  E6  Within-subject direction. For the three people who used both rigs, the
      sign of each person's peakGyro change predicts the sign of their
      rules-only twist change.
  E7  Per-participant consistency. Each swap and the fitted rules, scored
      separately for every one of the ten wrist participants, with an exact
      sign test across participants.
  E8  Quantile prediction. Each boundary's class-kept fraction on the wrist,
      predicted by scaling every breadboard window (not just the median).
  E9  Axis mixing. The class-wise change in `ratio` (axis concentration),
      added to the gain replay, to account for the twist loss gain misses.

Usage:
  python3 research/mechanism.py               # all experiments
  python3 research/mechanism.py --markdown    # paper-ready tables
  python3 research/mechanism.py --figures     # writes the three figures
  python3 research/mechanism.py --json out.json
"""

import argparse
import json
import math
import os
import random
import statistics as st
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, ".."))
sys.path.insert(0, _HERE)

import lyra_ml as core            # noqa: E402  research/lyra_ml.py (engine)
import firmware_port as fw        # noqa: E402  deployed-constant reference

BB_CSV = os.path.join(_ROOT, "data", "lyra_multisubject.csv")
WR_CSV = os.path.join(_ROOT, "data", "real_10subj.csv")
FIGDIR = os.path.join(_ROOT, "figures")

# Complete breadboard subjects (the LOSO folds) and their wrist identities.
# A, B, C in the breadboard data are the same people as A, B, C in the wrist
# data (P1-P3 in the paper).
BB_SUBJECTS = ["A", "B", "C"]
TRIAD = {"A": ("A", "A"), "B": ("B", "B"), "C": ("C", "C")}

# Hand-set rule constants (lyra_calibration_stream_new.ino 84, 89).
HAND_SEP = 700.0     # PINCH_TWIST_BOUNDARY, "FIXED (project decision)"
HAND_GATE = 0.40     # pinchGate / MIN_PINCH_ACCEL_STD, "provisional"

GYRO_KEYS = ("peakGyro", "gySigned", "gzSigned", "absGy", "absGz")
GESTURES4 = ("TWIST", "PINCH", "FLICK", "NONE")
SEED = 0
N_BOOT = 2000
N_TREES = 300


# --------------------------------------------------------------------------- #
# Data
# --------------------------------------------------------------------------- #
def _with_abs(r):
    x = dict(r)
    x["absGy"] = abs(r["gySigned"])
    x["absGz"] = abs(r["gzSigned"])
    return x


def load_data():
    bb, _ = core.load(BB_CSV)
    wr, _ = core.load(WR_CSV)
    bb = [_with_abs(r) for r in bb if r["subject"] in BB_SUBJECTS]
    wr = [_with_abs(r) for r in wr]
    return bb, wr


# --------------------------------------------------------------------------- #
# Statistics
# --------------------------------------------------------------------------- #
def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


def mcnemar_exact(b, c):
    """Two-sided exact McNemar p from the discordant counts b, c."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def hits(rows, clf, g):
    return [clf(r) == g for r in rows if r["truth"] == g]


def acc(rows, clf, g):
    h = hits(rows, clf, g)
    return sum(h) / len(h)


def score(rows, clf):
    return {g: acc(rows, clf, g) for g in ("TWIST", "PINCH", "FLICK", "NONE")}


def paired(rows, clf_a, clf_b, g):
    """(b, c, p): b = A right & B wrong, c = A wrong & B right."""
    sub = [r for r in rows if r["truth"] == g]
    b = sum(1 for r in sub if clf_a(r) == g and clf_b(r) != g)
    c = sum(1 for r in sub if clf_a(r) != g and clf_b(r) == g)
    return b, c, mcnemar_exact(b, c)


def cluster_boot_diff(rows, clf_a, clf_b, g, n=N_BOOT, seed=SEED):
    """95% CI of acc(B) - acc(A), resampling whole subjects (windows from one
    person are not independent)."""
    rng = random.Random(seed)
    by = {}
    for r in rows:
        if r["truth"] == g:
            by.setdefault(r["subject"], []).append(
                (clf_a(r) == g, clf_b(r) == g))
    subs = sorted(by)
    diffs = []
    for _ in range(n):
        pick = [p for s in (rng.choice(subs) for _ in subs) for p in by[s]]
        diffs.append((sum(b for _, b in pick) - sum(a for a, _ in pick)) / len(pick))
    diffs.sort()
    return diffs[int(0.025 * n)], diffs[int(0.975 * n) - 1]


# --------------------------------------------------------------------------- #
# Classifier variants. Every variant shares the deployed stage-1 gates exactly
# (gyro floor 220, flick >= 700 dps with ratio > 1.20), so only the twist/pinch
# decision differs between them.
# --------------------------------------------------------------------------- #
def _stage1(r):
    if r["peakGyro"] < fw.GYRO_FLOOR:
        return "NONE"
    if r["peakGyro"] >= fw.FLICK_GYRO and r["ratio"] > fw.RATIO_CUT:
        return "FLICK"
    return None


def cascade(sep, gate, sep_thr=HAND_SEP, gate_thr=HAND_GATE):
    """The rule cascade with each of its two decisions selectable.

    sep:  "hand" -> short (pinch-like) if peakGyro < sep_thr   (rules)
          "tree" -> short if durationCount <= 18.5, and a long window must
                    clear peakGyro > 422.05 to be a twist      (deployed tree)
    gate: "hand" -> a short window is PINCH if accelStd >= gate_thr  (rules)
          "tree" -> a short window is PINCH if accelStd <= 0.692     (tree)
    """
    T = fw.TREE

    def run(r):
        s1 = _stage1(r)
        if s1:
            return s1
        short = (r["peakGyro"] < sep_thr if sep == "hand"
                 else r["durationCount"] <= T["durSplit"])
        if short:
            ok = (r["accelStd"] >= gate_thr if gate == "hand"
                  else r["accelStd"] <= T["aStdSplit"])
            return "PINCH" if ok else "NONE"
        if sep == "hand":
            return "TWIST"
        return "TWIST" if r["peakGyro"] > T["pgHighSplit"] else "NONE"
    return run


def deployed_with(root="tree", pinch="tree"):
    """The deployed stage-2 tree, with its root split and/or its pinch leaf
    replaced by the rules' version (the reverse direction of the swap)."""
    T = fw.TREE

    def run(r):
        s1 = _stage1(r)
        if s1:
            return s1
        short = (r["durationCount"] <= T["durSplit"] if root == "tree"
                 else r["peakGyro"] < HAND_SEP)
        if short:
            if r["accelMean"] <= T["aMeanSplit"]:
                if r["peakGyro"] <= T["pgLowSplit"]:
                    return "NONE"
                return "TWIST" if r["spikeCount"] <= T["spikeSplit"] else "NONE"
            ok = (r["accelStd"] <= T["aStdSplit"] if pinch == "tree"
                  else r["accelStd"] >= HAND_GATE)
            return "PINCH" if ok else "NONE"
        if root == "tree":
            return "NONE" if r["peakGyro"] <= T["pgHighSplit"] else "TWIST"
        return "TWIST"
    return run


def gini_split(rows, feat):
    ys = [r["truth"] for r in rows]
    return core.build_tree(rows, [feat], ys, 0, 1, 3).thr


def fit_rule_thresholds(bb):
    """Re-choose the rules' two numbers by a one-split Gini fit, keeping the
    rule structure (lower-bound gate direction included) unchanged."""
    resid = [r for r in bb if _stage1(r) is None]
    tp = [r for r in resid if r["truth"] in ("TWIST", "PINCH")]
    sep = gini_split(tp, "peakGyro")
    low = [r for r in resid if r["peakGyro"] < sep and r["truth"] in ("PINCH", "NONE")]
    gate = gini_split(low, "accelStd")
    return sep, gate


# --------------------------------------------------------------------------- #
# E3: critical gain per boundary
# --------------------------------------------------------------------------- #
def boundaries(fit_sep, fit_gate):
    T = fw.TREE
    # (classifier, boundary, feature, threshold, protected class, kept-if)
    return [
        ("Rules", "twist/pinch separator", "peakGyro", HAND_SEP, "TWIST", ">="),
        ("Rules", "pinch gate", "accelStd", HAND_GATE, "PINCH", ">="),
        ("Tree", "root, twist side", "durationCount", T["durSplit"], "TWIST", ">"),
        ("Tree", "root, pinch side", "durationCount", T["durSplit"], "PINCH", "<="),
        ("Tree", "twist floor", "peakGyro", T["pgHighSplit"], "TWIST", ">"),
        ("Tree", "pinch noise cap", "accelStd", T["aStdSplit"], "PINCH", "<="),
        ("Fitted rules", "twist/pinch separator", "peakGyro", fit_sep, "TWIST", ">="),
        ("Fitted rules", "pinch gate", "accelStd", fit_gate, "PINCH", ">="),
    ]


def critical_gain_table(bb, wr, fit_sep, fit_gate):
    out = []
    for clf, name, f, thr, g, side in boundaries(fit_sep, fit_gate):
        m_bb = st.median(r[f] for r in bb if r["truth"] == g)
        m_wr = st.median(r[f] for r in wr if r["truth"] == g)
        k_star = thr / m_bb
        k_obs = m_wr / m_bb
        keep = (lambda v: v >= thr) if side in (">=", ">") else (lambda v: v <= thr)
        # A lower bound fails when the gain falls below k*; an upper bound when
        # it rises above k*.
        crosses = k_obs < k_star if side in (">=", ">") else k_obs > k_star
        pass_bb = sum(keep(r[f]) for r in bb if r["truth"] == g) / sum(r["truth"] == g for r in bb)
        pass_wr = sum(keep(r[f]) for r in wr if r["truth"] == g) / sum(r["truth"] == g for r in wr)
        # E8: the whole distribution, not just the median. Scale every
        # breadboard window of the class by k_obs and count how many the
        # boundary still keeps; this predicts the size of the loss, which the
        # median rule (half the class crosses at k*) cannot.
        vals = [r[f] for r in bb if r["truth"] == g]
        pass_pred = sum(keep(v * k_obs) for v in vals) / len(vals)
        out.append({"classifier": clf, "boundary": name, "feature": f,
                    "threshold": thr, "class": g, "side": side,
                    "median_bb": m_bb, "k_star": k_star, "k_obs": k_obs,
                    "predicted_fail": crosses,
                    "pass_bb": pass_bb, "pass_wr": pass_wr, "pass_pred": pass_pred})
    return out


# --------------------------------------------------------------------------- #
# E4: synthetic gain
# --------------------------------------------------------------------------- #
def rescale(rows, ka=1.0, kg=1.0):
    """Multiply accelerometer deviation by ka and gyro amplitude by kg.
    durationCount, spikeCount, ratio and accelMean are held fixed (ratio is a
    ratio of gyro peaks, so it is invariant to a common kg by construction)."""
    out = []
    for r in rows:
        x = dict(r)
        x["accelStd"] *= ka
        x["peakAccel"] = 1.0 + (x["peakAccel"] - 1.0) * ka
        for k in GYRO_KEYS:
            x[k] *= kg
        out.append(x)
    return out


def remix(rows, kr):
    """E9: axis mixing. Multiply each window's axis-concentration `ratio` by
    its class's observed wrist/breadboard ratio gain kr[class]. A wrist mount
    aligns a gyro axis with the forearm, so rotation about it (twist)
    concentrates in one axis and `ratio` rises; a pure amplitude gain cannot
    change `ratio`, so this is the part of the shift the gain model misses.
    Uses truth labels, so it is a diagnostic, not a deployable correction."""
    out = []
    for r in rows:
        x = dict(r)
        x["ratio"] *= kr[r["truth"]]
        out.append(x)
    return out


def observed_gain(bb, wr, feat):
    """Median over gesture classes of (wrist median / breadboard median)."""
    return st.median(
        st.median(r[feat] for r in wr if r["truth"] == g)
        / st.median(r[feat] for r in bb if r["truth"] == g)
        for g in ("TWIST", "PINCH", "FLICK"))


# --------------------------------------------------------------------------- #
# E5: learned trees in general
# --------------------------------------------------------------------------- #
AMPLITUDE = ["peakGyro", "peakAccel", "accelStd", "gySigned", "gzSigned"]
NON_AMPLITUDE = ["durationCount", "spikeCount", "ratio", "accelMean"]


def hybrid_from(node):
    def run(r):
        s1 = _stage1(r)
        return s1 if s1 else core.tree_predict(node, r)
    return run


def fit_stage2(rows, feats, depth=4, min_leaf=3):
    resid = [r for r in rows if _stage1(r) is None]
    return core.build_tree(resid, feats, [r["truth"] for r in resid], 0, depth, min_leaf)


def loso_score(bb, feats):
    pairs = core.loso(bb, BB_SUBJECTS,
                      lambda tr, te: [hybrid_from(fit_stage2(tr, feats))(r) for r in te])
    out = {}
    for g in ("TWIST", "PINCH", "NONE"):
        s = [(t, p) for t, p in pairs if t == g]
        out[g] = sum(t == p for t, p in s) / len(s)
    return out


def split_feats(node, acc_=None):
    acc_ = [] if acc_ is None else acc_
    if node.label is None:
        acc_.append(node.feat)
        split_feats(node.left, acc_)
        split_feats(node.right, acc_)
    return acc_


def bootstrap_trees(bb, wr, ka, kg, n=N_TREES, seed=SEED):
    """Each tree is scored on the real wrist data and, as an out-of-sample test
    of the gain hypothesis, on breadboard data rescaled by the observed gain
    (computed before any tree is fit, from feature medians only)."""
    rng = random.Random(seed)
    resid = [r for r in bb if _stage1(r) is None]
    sim = rescale(bb, ka=ka, kg=kg)
    res = []
    for _ in range(n):
        s = [rng.choice(resid) for _ in resid]
        node = core.build_tree(s, core.FEATURES, [r["truth"] for r in s], 0, 4, 3)
        clf = hybrid_from(node)
        sc = score(wr, clf)
        res.append({"root": node.feat, "TWIST": sc["TWIST"], "PINCH": sc["PINCH"],
                    "NONE": sc["NONE"], "sim_TWIST": acc(sim, clf, "TWIST"),
                    "sim_PINCH": acc(sim, clf, "PINCH")})
    return res


def spearman(x, y):
    def ranks(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(v):
            j = i
            while j + 1 < len(v) and v[order[j + 1]] == v[order[i]]:
                j += 1
            for k in range(i, j + 1):
                r[order[k]] = (i + j) / 2.0
            i = j + 1
        return r
    rx, ry = ranks(x), ranks(y)
    mx, my = st.mean(rx), st.mean(ry)
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    den = math.sqrt(sum((a - mx) ** 2 for a in rx) * sum((b - my) ** 2 for b in ry))
    return num / den if den else float("nan")


def pctl(v, p):
    v = sorted(v)
    return v[int(round(p * (len(v) - 1)))]


# --------------------------------------------------------------------------- #
# Run everything
# --------------------------------------------------------------------------- #
def run_all(bb, wr):
    R = {}
    rules = fw.classify_fast
    hybrid = fw.classify_hybrid

    # Sanity: the variants built here reproduce both deployed classifiers.
    for rows in (bb, wr):
        assert all(cascade("hand", "hand")(r) == rules(r) for r in rows)
        assert all(deployed_with()(r) == hybrid(r) for r in rows)

    # ---- E1 component swap ---------------------------------------------------
    grid = {}
    for sep in ("hand", "tree"):
        for gate in ("hand", "tree"):
            c = cascade(sep, gate)
            grid[(sep, gate)] = {"bb": score(bb, c), "wr": score(wr, c)}
    rev = {
        "deployed": deployed_with(),
        "root -> rules' peakGyro<700": deployed_with(root="hand"),
        "pinch leaf -> rules' accelStd>=0.40": deployed_with(pinch="hand"),
    }
    R["swap_grid"] = grid
    R["reverse"] = {k: {"bb": score(bb, c), "wr": score(wr, c)} for k, c in rev.items()}

    tests = []
    for label, a, b, g in [
            ("rules -> tree separator", cascade("hand", "hand"), cascade("tree", "hand"), "TWIST"),
            ("rules -> tree separator", cascade("hand", "hand"), cascade("tree", "hand"), "PINCH"),
            ("rules -> tree pinch gate", cascade("hand", "hand"), cascade("hand", "tree"), "PINCH"),
            ("rules -> tree pinch gate", cascade("hand", "hand"), cascade("hand", "tree"), "TWIST"),
            ("tree -> rules root", deployed_with(), deployed_with(root="hand"), "TWIST"),
            ("tree -> rules pinch leaf", deployed_with(), deployed_with(pinch="hand"), "PINCH")]:
        bcount, ccount, p = paired(wr, a, b, g)
        lo, hi = cluster_boot_diff(wr, a, b, g)
        tests.append({"swap": label, "gesture": g, "acc_before": acc(wr, a, g),
                      "acc_after": acc(wr, b, g), "b": bcount, "c": ccount,
                      "p": p, "ci_lo": lo, "ci_hi": hi})
    R["swap_tests"] = tests

    # ---- E2 fitted thresholds ----------------------------------------------
    fit_sep, fit_gate = fit_rule_thresholds(bb)
    fitted = cascade("hand", "hand", fit_sep, fit_gate)
    R["fitted"] = {"sep": fit_sep, "gate": fit_gate,
                   "bb": score(bb, fitted), "wr": score(wr, fitted)}
    R["fitted_tests"] = []
    for g in ("TWIST", "PINCH"):
        bcount, ccount, p = paired(wr, rules, fitted, g)
        lo, hi = cluster_boot_diff(wr, rules, fitted, g)
        R["fitted_tests"].append({"gesture": g, "rules": acc(wr, rules, g),
                                  "fitted": acc(wr, fitted, g), "b": bcount,
                                  "c": ccount, "p": p, "ci_lo": lo, "ci_hi": hi})
    R["rules"] = {"bb": score(bb, rules), "wr": score(wr, rules)}
    R["hybrid"] = {"bb": score(bb, hybrid), "wr": score(wr, hybrid)}

    # ---- E3 critical gain ---------------------------------------------------
    R["critical_gain"] = critical_gain_table(bb, wr, fit_sep, fit_gate)

    # ---- E4 sweeps + replay -------------------------------------------------
    ka_obs = observed_gain(bb, wr, "accelStd")
    kg_obs = observed_gain(bb, wr, "peakGyro")
    ka_grid = [round(0.5 + 0.05 * i, 2) for i in range(27)]      # 0.50 .. 1.80
    kg_grid = [round(0.6 + 0.05 * i, 2) for i in range(15)]      # 0.60 .. 1.30
    R["sweep_ka"] = [{"k": k, "rules": score(rescale(bb, ka=k), rules),
                      "hybrid": score(rescale(bb, ka=k), hybrid)} for k in ka_grid]
    R["sweep_kg"] = [{"k": k, "rules": score(rescale(bb, kg=k), rules),
                      "hybrid": score(rescale(bb, kg=k), hybrid)} for k in kg_grid]
    rep = rescale(bb, ka=ka_obs, kg=kg_obs)
    R["replay"] = {"ka": ka_obs, "kg": kg_obs,
                   "rules_sim": score(rep, rules), "hybrid_sim": score(rep, hybrid),
                   "rules_wr": score(wr, rules), "hybrid_wr": score(wr, hybrid)}
    # Twists that leave via the flick gate are a non-gain effect (axis mix).
    # ---- E9 axis mixing on top of the gain replay -----------------------------
    kr = {g: st.median(r["ratio"] for r in wr if r["truth"] == g)
          / st.median(r["ratio"] for r in bb if r["truth"] == g) for g in GESTURES4}
    mixed = remix(rep, kr)

    def to_flick(rows):
        s = [r for r in rows if r["truth"] == "TWIST"]
        return sum(rules(r) == "FLICK" for r in s) / len(s)
    rb, rw = acc(bb, rules, "TWIST"), acc(wr, rules, "TWIST")
    R["axis_mixing"] = {
        "kr": kr,
        "rules_gain_only": score(rep, rules), "rules_gain_mix": score(mixed, rules),
        "hybrid_gain_mix": score(mixed, hybrid),
        "twist_loss_explained_gain": (rb - acc(rep, rules, "TWIST")) / (rb - rw),
        "twist_loss_explained_mix": (rb - acc(mixed, rules, "TWIST")) / (rb - rw),
        "twist_to_flick": {"bb": to_flick(bb), "gain": to_flick(rep),
                           "gain_mix": to_flick(mixed), "wr": to_flick(wr)}}

    R["twist_to_flick_wr"] = sum(1 for r in wr if r["truth"] == "TWIST"
                                 and _stage1(r) == "FLICK")
    R["ratio_twist"] = {"bb": st.median(r["ratio"] for r in bb if r["truth"] == "TWIST"),
                        "wr": st.median(r["ratio"] for r in wr if r["truth"] == "TWIST")}

    # ---- E5 learned trees ---------------------------------------------------
    subsets = {"All nine features": core.FEATURES,
               "Amplitude features only": AMPLITUDE,
               "Non-amplitude features only": NON_AMPLITUDE,
               "All but durationCount": [f for f in core.FEATURES if f != "durationCount"]}
    R["subsets"] = {}
    for name, feats in subsets.items():
        node = fit_stage2(bb, feats)
        R["subsets"][name] = {"loso_bb": loso_score(bb, feats),
                              "wr": score(wr, hybrid_from(node)),
                              "root": node.feat, "thr": node.thr}
    boots = bootstrap_trees(bb, wr, ka_obs, kg_obs)
    roots = {}
    for b in boots:
        roots[b["root"]] = roots.get(b["root"], 0) + 1
    # Does the gain model, fit to nothing but feature medians, predict which of
    # the 300 trees break on the wrist? "Break" = loses > 20 points on pinch.
    pin_sim = [b["sim_PINCH"] for b in boots]
    pin_wr = [b["PINCH"] for b in boots]
    pred_break = [s < 0.5 for s in pin_sim]
    real_break = [w < 0.5 for w in pin_wr]
    R["bootstrap_prediction"] = {
        "spearman_pinch": spearman(pin_sim, pin_wr),
        "spearman_twist": spearman([b["sim_TWIST"] for b in boots],
                                   [b["TWIST"] for b in boots]),
        "n_pred_break": sum(pred_break), "n_real_break": sum(real_break),
        "both": sum(p and r for p, r in zip(pred_break, real_break)),
        "agree": sum(p == r for p, r in zip(pred_break, real_break)),
        "pairs": [(s, w) for s, w in zip(pin_sim, pin_wr)]}
    R["bootstrap"] = {
        "n": len(boots), "roots": roots,
        **{g: {"median": st.median(b[g] for b in boots),
               "p05": pctl([b[g] for b in boots], 0.05),
               "min": min(b[g] for b in boots)} for g in ("TWIST", "PINCH", "NONE")},
        "rules_wr": score(wr, rules)}

    # ---- E6 within-subject direction ---------------------------------------
    tri = []
    for s, (bb_id, wr_id) in TRIAD.items():
        a = [r for r in bb if r["subject"] == s and r["truth"] == "TWIST"]
        b = [r for r in wr if r["subject"] == wr_id and r["truth"] == "TWIST"]
        kg_s = st.median(r["peakGyro"] for r in b) / st.median(r["peakGyro"] for r in a)
        ka_s = st.median(r["accelStd"] for r in b) / st.median(r["accelStd"] for r in a)
        ra = sum(rules(r) == "TWIST" for r in a) / len(a)
        rb = sum(rules(r) == "TWIST" for r in b) / len(b)
        tri.append({"person": bb_id, "kg": kg_s, "ka": ka_s,
                    "frac_above_700_bb": sum(r["peakGyro"] >= HAND_SEP for r in a) / len(a),
                    "frac_above_700_wr": sum(r["peakGyro"] >= HAND_SEP for r in b) / len(b),
                    "rules_twist_bb": ra, "rules_twist_wr": rb,
                    "predicted_up": kg_s > 1.0, "observed_up": rb > ra})
    R["triad"] = tri

    # ---- E7 per-participant consistency (all ten wrist participants) --------
    variants = {"rules": rules, "tree separator": cascade("tree", "hand"),
                "tree pinch gate": cascade("hand", "tree"), "fitted rules": fitted,
                "hybrid": hybrid}
    per = []
    for s in sorted(set(r["subject"] for r in wr)):
        rows = [r for r in wr if r["subject"] == s]
        row = {"participant": s}
        for g in ("TWIST", "PINCH"):
            row[g] = {k: acc(rows, c, g) for k, c in variants.items()}
            row[g]["n"] = sum(r["truth"] == g for r in rows)
        per.append(row)
    n = len(per)

    def sign_test(k):
        """Two-sided exact sign test: k of n participants improved."""
        tail = sum(math.comb(n, i) for i in range(k, n + 1)) / 2 ** n
        return min(1.0, 2 * tail)
    wins = {
        "tree separator (twist)": sum(p["TWIST"]["tree separator"] > p["TWIST"]["rules"] for p in per),
        "tree pinch gate (pinch)": sum(p["PINCH"]["tree pinch gate"] > p["PINCH"]["rules"] for p in per),
        "fitted rules (twist)": sum(p["TWIST"]["fitted rules"] > p["TWIST"]["rules"] for p in per),
        "fitted rules (pinch)": sum(p["PINCH"]["fitted rules"] > p["PINCH"]["rules"] for p in per),
    }
    R["per_participant"] = per
    R["per_participant_wins"] = {k: {"k": v, "n": n, "p": sign_test(v)} for k, v in wins.items()}
    return R


# --------------------------------------------------------------------------- #
# Output
# --------------------------------------------------------------------------- #
def p100(x):
    return f"{100 * x:.1f}%"


def fmt_p(p):
    return "< 0.0001" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.4f}"


def print_text(R):
    print("\n=== Baselines (deployed constants, recomputed from raw features) ===")
    for k in ("rules", "hybrid"):
        for d in ("bb", "wr"):
            s = R[k][d]
            print(f"  {k:7s} {d}: twist {p100(s['TWIST'])}  pinch {p100(s['PINCH'])}"
                  f"  flick {p100(s['FLICK'])}  NONE rejected {p100(s['NONE'])}")

    print("\n=== E1 component swap (2x2): separator x pinch gate ===")
    for (sep, gate), v in R["swap_grid"].items():
        print(f"  sep={sep:4s} gate={gate:4s}  BB twist {p100(v['bb']['TWIST'])} "
              f"pinch {p100(v['bb']['PINCH'])}  |  WR twist {p100(v['wr']['TWIST'])} "
              f"pinch {p100(v['wr']['PINCH'])}  NONE {p100(v['wr']['NONE'])}")
    for k, v in R["reverse"].items():
        print(f"  deployed tree, {k:36s} WR twist {p100(v['wr']['TWIST'])} "
              f"pinch {p100(v['wr']['PINCH'])}")
    for t in R["swap_tests"]:
        print(f"  {t['swap']:26s} {t['gesture']:5s} {p100(t['acc_before'])} -> "
              f"{p100(t['acc_after'])}  McNemar b={t['b']} c={t['c']} p={fmt_p(t['p'])}"
              f"  subject-bootstrap CI [{100*t['ci_lo']:+.1f}, {100*t['ci_hi']:+.1f}] pts")

    f = R["fitted"]
    print(f"\n=== E2 same rule structure, Gini-fitted thresholds: "
          f"separator {f['sep']:.2f} dps (hand 700), gate {f['gate']:.3f} g (hand 0.40) ===")
    print(f"  BB twist {p100(f['bb']['TWIST'])} pinch {p100(f['bb']['PINCH'])} "
          f"NONE {p100(f['bb']['NONE'])} | WR twist {p100(f['wr']['TWIST'])} "
          f"pinch {p100(f['wr']['PINCH'])} NONE {p100(f['wr']['NONE'])}")
    for t in R["fitted_tests"]:
        print(f"  {t['gesture']}: rules {p100(t['rules'])} -> fitted {p100(t['fitted'])}"
              f"  McNemar p={fmt_p(t['p'])}  CI [{100*t['ci_lo']:+.1f}, {100*t['ci_hi']:+.1f}]")

    print("\n=== E3 critical gain k* vs observed gain ===")
    for b in R["critical_gain"]:
        print(f"  {b['classifier']:12s} {b['boundary']:22s} {b['feature']:13s} "
              f"thr {b['threshold']:8.3f} k*={b['k_star']:.2f} obs={b['k_obs']:.2f} "
              f"predict {'FAIL' if b['predicted_fail'] else 'hold'}  "
              f"pass {p100(b['pass_bb'])} -> {p100(b['pass_wr'])} (E8 predicted {p100(b['pass_pred'])})")
    err = [abs(b["pass_pred"] - b["pass_wr"]) for b in R["critical_gain"]]
    print(f"  E8 mean |predicted - actual| class kept: {100*st.mean(err):.1f} points")

    rp = R["replay"]
    print(f"\n=== E4 replay of observed gain (ka={rp['ka']:.3f}, kg={rp['kg']:.3f}) ===")
    for k in ("rules", "hybrid"):
        s, w = rp[k + "_sim"], rp[k + "_wr"]
        print(f"  {k:7s} simulated twist {p100(s['TWIST'])} pinch {p100(s['PINCH'])}"
              f"   actual wrist twist {p100(w['TWIST'])} pinch {p100(w['PINCH'])}")
    print(f"  wrist twists sent to FLICK by stage 1: {R['twist_to_flick_wr']} "
          f"(twist ratio median {R['ratio_twist']['bb']:.2f} -> {R['ratio_twist']['wr']:.2f})")
    print("  ka sweep (rules pinch / hybrid pinch):",
          " ".join(f"{s['k']}:{100*s['rules']['PINCH']:.0f}/{100*s['hybrid']['PINCH']:.0f}"
                   for s in R["sweep_ka"][::2]))
    print("  kg sweep (rules twist / hybrid twist):",
          " ".join(f"{s['k']}:{100*s['rules']['TWIST']:.0f}/{100*s['hybrid']['TWIST']:.0f}"
                   for s in R["sweep_kg"]))

    am = R["axis_mixing"]
    print("\n=== E9 axis mixing (ratio gain per class) on top of the gain replay ===")
    print("  ratio gain: " + "  ".join(f"{g.lower()} x{k:.2f}" for g, k in am["kr"].items()))
    for lab, k in [("gain only", "rules_gain_only"), ("gain + mixing", "rules_gain_mix")]:
        s_ = am[k]
        print(f"  rules, {lab:14s} twist {p100(s_['TWIST'])} pinch {p100(s_['PINCH'])}"
              f" flick {p100(s_['FLICK'])}")
    print(f"  share of rules' twist loss explained: gain {100*am['twist_loss_explained_gain']:.0f}%,"
          f" gain + mixing {100*am['twist_loss_explained_mix']:.0f}%")
    tf = am["twist_to_flick"]
    print(f"  twists called FLICK: breadboard {p100(tf['bb'])}, gain {p100(tf['gain'])},"
          f" gain + mixing {p100(tf['gain_mix'])}, actual wrist {p100(tf['wr'])}")
    h = am["hybrid_gain_mix"]
    print(f"  hybrid, gain + mixing: twist {p100(h['TWIST'])} pinch {p100(h['PINCH'])}")

    print("\n=== E5 learned trees fit on breadboard, scored on wrist ===")
    for k, v in R["subsets"].items():
        print(f"  {k:28s} root {v['root']:13s} BB-LOSO twist {p100(v['loso_bb']['TWIST'])}"
              f" pinch {p100(v['loso_bb']['PINCH'])} | WR twist {p100(v['wr']['TWIST'])}"
              f" pinch {p100(v['wr']['PINCH'])} NONE {p100(v['wr']['NONE'])}")
    b = R["bootstrap"]
    print(f"  {b['n']} bootstrap trees, roots {b['roots']}")
    for g in ("TWIST", "PINCH", "NONE"):
        print(f"    WR {g}: median {p100(b[g]['median'])}, 5th pct {p100(b[g]['p05'])},"
              f" worst {p100(b[g]['min'])}  (rules {p100(b['rules_wr'][g])})")
    bp = R["bootstrap_prediction"]
    print(f"  gain model predicting each tree's wrist accuracy: Spearman pinch "
          f"{bp['spearman_pinch']:.2f}, twist {bp['spearman_twist']:.2f}")
    print(f"  pinch < 50%: predicted {bp['n_pred_break']}, actual {bp['n_real_break']}, "
          f"both {bp['both']}; agreement {bp['agree']}/{len(bp['pairs'])}")

    print("\n=== E6 within-subject: sign of peakGyro change predicts rules-twist change ===")
    for t in R["triad"]:
        print(f"  {t['person']}: kg={t['kg']:.2f} ka={t['ka']:.2f}  >=700 dps "
              f"{p100(t['frac_above_700_bb'])} -> {p100(t['frac_above_700_wr'])}  rules twist "
              f"{p100(t['rules_twist_bb'])} -> {p100(t['rules_twist_wr'])}  "
              f"predicted {'up' if t['predicted_up'] else 'down'}, observed "
              f"{'up' if t['observed_up'] else 'down'}")

    print("\n=== E7 per wrist participant: swap / fitted-rules improvement ===")
    for p in R["per_participant"]:
        t, q = p["TWIST"], p["PINCH"]
        print(f"  {p['participant']}: twist rules {100*t['rules']:.0f} -> sep {100*t['tree separator']:.0f}"
              f" / fitted {100*t['fitted rules']:.0f} (n={t['n']})   pinch rules {100*q['rules']:.0f}"
              f" -> gate {100*q['tree pinch gate']:.0f} / fitted {100*q['fitted rules']:.0f} (n={q['n']})")
    for k, v in R["per_participant_wins"].items():
        print(f"  {k}: improved in {v['k']}/{v['n']} participants, sign test p = {fmt_p(v['p'])}")


def print_markdown(R):
    print("\n**Table 7.** Component swap (2×2), wrist data. Each cell changes only "
          "the twist/pinch separator and/or the pinch gate; stage 1 is identical.\n")
    print("| Twist/pinch separator | Pinch gate | Twist (wrist) | Pinch (wrist) | NONE rejected (wrist) |")
    print("|---|---|---|---|---|")
    name = {"hand": ("Rules: peakGyro < 700", "Rules: accelStd ≥ 0.40"),
            "tree": ("Tree: durationCount ≤ 18.5", "Tree: accelStd ≤ 0.692")}
    for (sep, gate), v in R["swap_grid"].items():
        w = v["wr"]
        print(f"| {name[sep][0]} | {name[gate][1]} | {p100(w['TWIST'])} | "
              f"{p100(w['PINCH'])} | {p100(w['NONE'])} |")
    print("\n**Table 8.** Rules with fitted thresholds.\n")
    print("| Classifier | Separator | Gate | Twist BB | Pinch BB | Twist wrist | Pinch wrist | NONE wrist |")
    print("|---|---|---|---|---|---|---|---|")
    for lab, s, g, d in [("Rules, hand-set", HAND_SEP, HAND_GATE, R["rules"]),
                         ("Rules, Gini-fitted", R["fitted"]["sep"], R["fitted"]["gate"], R["fitted"])]:
        print(f"| {lab} | {s:.1f} dps | {g:.3f} g | {p100(d['bb']['TWIST'])} | "
              f"{p100(d['bb']['PINCH'])} | {p100(d['wr']['TWIST'])} | "
              f"{p100(d['wr']['PINCH'])} | {p100(d['wr']['NONE'])} |")
    print("\n**Table 9.** Critical gain per decision boundary.\n")
    print("| Classifier | Boundary | Feature | Threshold | Protected class (BB median) | k* | Observed gain | Predicted | Class kept, BB → wrist |")
    print("|---|---|---|---|---|---|---|---|---|")
    for b in R["critical_gain"]:
        print(f"| {b['classifier']} | {b['boundary']} | `{b['feature']}` {b['side']} | "
              f"{b['threshold']:.3g} | {b['class'].lower()} ({b['median_bb']:.3g}) | "
              f"{b['k_star']:.2f} | {b['k_obs']:.2f} | "
              f"{'**fails**' if b['predicted_fail'] else 'holds'} | "
              f"{p100(b['pass_bb'])} → {p100(b['pass_wr'])} |")


# --------------------------------------------------------------------------- #
# Figures
# --------------------------------------------------------------------------- #
def make_figures(R):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(FIGDIR, exist_ok=True)
    RULES_C, HYB_C = "#9e9ac8", "#d95f02"

    def save(fig, name):
        path = os.path.join(FIGDIR, name)
        fig.savefig(path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print(f"wrote {path}")

    # component swap
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.8), sharey=True)
    labels = ["Rules\n(hand-set)", "Tree\nseparator", "Tree\npinch gate",
              "Both\nfrom tree"]
    keys = [("hand", "hand"), ("tree", "hand"), ("hand", "tree"), ("tree", "tree")]
    for ax, g, col in [(axes[0], "TWIST", "#d95f02"), (axes[1], "PINCH", "#1b9e77")]:
        bbv = [100 * R["swap_grid"][k]["bb"][g] for k in keys]
        wrv = [100 * R["swap_grid"][k]["wr"][g] for k in keys]
        xs = range(len(keys))
        ax.bar([x - 0.2 for x in xs], bbv, 0.38, color="#cfcfcf", label="Breadboard")
        bars = ax.bar([x + 0.2 for x in xs], wrv, 0.38, color=col, label="Wrist")
        for b_, v in zip(bars, wrv):
            ax.text(b_.get_x() + b_.get_width() / 2, v + 1.5, f"{v:.0f}",
                    ha="center", fontsize=8)
        ax.set_xticks(list(xs), labels, fontsize=8)
        ax.set_ylim(0, 108)
        ax.set_title(f"{'AB'[g == 'PINCH']}  {g.lower()}", fontsize=10, loc="left")
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("accuracy (%)")
    axes[1].legend(fontsize=8, frameon=False, loc="upper left")
    save(fig, "mechanism_component_swap.png")

    # gain sweeps
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.8), sharey=True)
    for ax, key, g, obs, xl in [
            (axes[0], "sweep_ka", "PINCH", R["replay"]["ka"],
             "accelerometer-deviation gain ka (×)"),
            (axes[1], "sweep_kg", "TWIST", R["replay"]["kg"], "gyro-amplitude gain kg (×)")]:
        ks = [s["k"] for s in R[key]]
        ax.plot(ks, [100 * s["rules"][g] for s in R[key]], "o-", ms=3,
                color=RULES_C, label="Rules (hand-set)")
        ax.plot(ks, [100 * s["hybrid"][g] for s in R[key]], "s-", ms=3,
                color=HYB_C, label="Hybrid (deployed)")
        ax.axvline(1.0, color="#999", lw=0.8)
        ax.axvline(obs, color="k", ls="--", lw=1)
        ax.text(obs, 103, f" observed\n wrist ×{obs:.2f}", fontsize=7, va="top")
        ax.set_xlabel(xl, fontsize=9)
        ax.set_title(f"{'AB'[g == 'TWIST']}  {g.lower()}", fontsize=10, loc="left")
        ax.set_ylim(0, 105)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("accuracy (%)")
    axes[0].legend(fontsize=8, frameon=False, loc="lower right")
    save(fig, "mechanism_gain_sweep.png")

    # critical gain vs observed
    cg = R["critical_gain"]
    fig, ax = plt.subplots(figsize=(7.2, 3.9))
    for i, b in enumerate(cg):
        y = len(cg) - 1 - i
        lower = b["side"] in (">=", ">")
        col = "#c0392b" if b["predicted_fail"] else "#2e7d32"
        lo, hi = (b["k_star"], 3.0) if lower else (0.3, b["k_star"])
        ax.plot([lo, hi], [y, y], color=col, lw=6, alpha=0.25, solid_capstyle="butt")
        ax.plot([b["k_star"]], [y], "|", color=col, ms=14, mew=2)
        ax.plot([b["k_obs"]], [y], "o", color="k", ms=5)
    ax.set_yticks(range(len(cg)),
                  [f"{b['classifier']}: {b['boundary']}" for b in reversed(cg)], fontsize=8)
    ax.axvline(1.0, color="#999", lw=0.8)
    ax.set_xscale("log")
    ax.minorticks_off()
    ax.set_xlim(0.3, 3.0)
    ax.set_xticks([0.3, 0.5, 0.7, 1, 1.5, 2, 3], ["0.3", "0.5", "0.7", "1", "1.5", "2", "3"])
    ax.set_xlabel("gain in the boundary's feature, wrist / breadboard (log scale)", fontsize=9)
    ax.spines[["top", "right"]].set_visible(False)
    save(fig, "mechanism_critical_gain.png")


def to_jsonable(R):
    out = dict(R)
    out["swap_grid"] = {f"sep={s},gate={g}": v for (s, g), v in R["swap_grid"].items()}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--markdown", action="store_true", help="paper-ready tables")
    ap.add_argument("--figures", action="store_true", help="write the three mechanism figures")
    ap.add_argument("--json", metavar="OUT", help="dump every number as JSON")
    args = ap.parse_args()

    bb, wr = load_data()
    print(f"Breadboard: {len(bb)} windows ({', '.join(BB_SUBJECTS)});"
          f" wrist: {len(wr)} windows ({len(set(r['subject'] for r in wr))} subjects)")
    R = run_all(bb, wr)
    print_text(R)
    if args.markdown:
        print_markdown(R)
    if args.figures:
        make_figures(R)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(to_jsonable(R), f, indent=2)
        print(f"wrote {args.json}")


if __name__ == "__main__":
    main()
