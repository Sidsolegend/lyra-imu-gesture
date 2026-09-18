#!/usr/bin/env python3
"""
firmware_port.py — a line-faithful Python port of the LYRA firmware DSP path,
used as the reference model for unit tests and figures.

Ports (with original file:line provenance):
  * extract_features()  <- newlyra_final.ino analyzeAndAct() 346-368 /
                           optimized lyra_features.h extractFeatures()
  * count_above_floor / count_accel_spikes  <- newlyra_final.ino 197-214
  * stage2 / classify_hybrid (DEPLOYED constants, incl. the 422.05 high-branch
    split and the spikeCount<=2.5 refinement)  <- newlyra_final.ino 219-243
  * classify_fast (the cascade that produced the CSV `classified` column)
                        <- optimized lyra_classifier.h classifyFast()
  * GTemplate binary layout + custom-gesture matcher <- newlyra_final.ino 85-91,
    286-304 (struct layout: 96 bytes, 3 pad bytes after `action`)

Nothing here trains anything: it exists so Python tests can assert that the
maths the firmware runs is the maths we think it runs, on both synthetic
windows and the committed dataset.
"""

import math
import struct

# ---- fixed constants (newlyra_final.ino 51-74) ------------------------------
ACCEL_SCALE = 8192.0          # +/-4 g
GYRO_SCALE = 16.4             # +/-2000 dps
WINDOW = 60
TRIGGER_THRESHOLD = 1.35
GYRO_FLOOR = 220.0            # MIN_GYRO_FLOOR default
FLICK_GYRO = 700.0            # FIXED by design
RATIO_CUT = 1.20              # default

# Deployed stage-2 tree splits (newlyra_final.ino stage2(), 222-232).
TREE = {
    "durSplit": 18.5,
    "aMeanSplit": 1.159,
    "pgLowSplit": 354.8,
    "spikeSplit": 2.5,
    "aStdSplit": 0.692,
    "pgHighSplit": 422.05,
}

FEATURE_ORDER = ["peakGyro", "ratio", "durationCount", "absGy", "absGz",
                 "peakAccel", "accelMean", "accelStd", "spikeCount"]


# ---- helpers (newlyra_final.ino 197-214) -------------------------------------
def count_above_floor(gx, gy, gz, floor):
    return sum(1 for i in range(len(gx))
               if max(abs(gx[i]), abs(gy[i]), abs(gz[i])) > floor)


def count_accel_spikes(buf, threshold):
    bursts, above = 0, False
    for v in buf:
        now_above = v > threshold
        if now_above and not above:
            bursts += 1
        above = now_above
    return bursts


# ---- feature extraction (newlyra_final.ino analyzeAndAct 346-368) -------------
def extract_features(accel, gx, gy, gz, gyro_floor=GYRO_FLOOR, dt=0.010,
                     dt_train=0.010):
    """Return the 9-feature dict + signed peaks, identical maths to firmware.

    dt/dt_train implement the optimized build's Δt count normalisation
    (lyra_features.h extractFeatures); dt == dt_train reproduces the original
    firmware bit-for-bit (scale factor 1.0).
    """
    assert len(accel) == len(gx) == len(gy) == len(gz)
    peak_gx = peak_gy = peak_gz = 0.0
    peak_gy_signed = peak_gz_signed = 0.0
    for i in range(len(gx)):
        if abs(gx[i]) > peak_gx:
            peak_gx = abs(gx[i])
        if abs(gy[i]) > peak_gy:
            peak_gy = abs(gy[i])
            peak_gy_signed = gy[i]
        if abs(gz[i]) > peak_gz:
            peak_gz = abs(gz[i])
        if abs(gz[i]) > abs(peak_gz_signed):
            peak_gz_signed = gz[i]
    peak_gyro = max(peak_gx, peak_gy, peak_gz)
    s = sorted([peak_gx, peak_gy, peak_gz], reverse=True)
    ratio = s[0] / (s[1] + s[2] + 0.001)      # same epsilon as firmware

    peak_accel = max(accel) if accel else 0.0
    n = len(accel)
    accel_mean = sum(accel) / n
    accel_var = sum((a - accel_mean) ** 2 for a in accel) / n   # population var
    accel_std = math.sqrt(accel_var)

    scale = dt / dt_train
    return {
        "peakGyro": peak_gyro,
        "ratio": ratio,
        "durationCount": count_above_floor(gx, gy, gz, gyro_floor) * scale,
        "absGy": peak_gy,
        "absGz": peak_gz,
        "peakAccel": peak_accel,
        "accelMean": accel_mean,
        "accelStd": accel_std,
        "spikeCount": count_accel_spikes(accel, TRIGGER_THRESHOLD) * scale,
        "peakGYSigned": peak_gy_signed,
        "peakGZSigned": peak_gz_signed,
    }


# ---- pre-trigger ring buffer (ROADMAP #8; mirrors lyra_features.h PreTrigRing) -
PRETRIGGER = 10


class PreTrigRing:
    """Fixed-size ring holding the last PRETRIGGER (accel, gx, gy, gz) samples,
    so a capture can include the gesture onset that precedes the 1.35 g trigger.
    Byte-for-byte the same semantics as the C++ PreTrigRing: push() overwrites
    oldest once full; drain() returns oldest->newest."""

    def __init__(self, size=PRETRIGGER):
        self.size = size
        self._buf = []

    def push(self, a, x, y, z):
        self._buf.append((a, x, y, z))
        if len(self._buf) > self.size:
            self._buf.pop(0)

    def reset(self):
        self._buf = []

    def drain(self):
        """Buffered samples, oldest first (list of (accel, gx, gy, gz))."""
        return list(self._buf)

    @property
    def count(self):
        return len(self._buf)


# ---- deployed classifier (newlyra_final.ino 219-243) --------------------------
def stage2(f, tree=TREE):
    if f["durationCount"] <= tree["durSplit"]:
        if f["accelMean"] <= tree["aMeanSplit"]:
            if f["peakGyro"] <= tree["pgLowSplit"]:
                return "NONE"
            return "TWIST" if f["spikeCount"] <= tree["spikeSplit"] else "NONE"
        return "PINCH" if f["accelStd"] <= tree["aStdSplit"] else "NONE"
    return "NONE" if f["peakGyro"] <= tree["pgHighSplit"] else "TWIST"


def classify_hybrid(f, gyro_floor=GYRO_FLOOR, flick_gyro=FLICK_GYRO,
                    ratio_cut=RATIO_CUT, tree=TREE):
    if f["peakGyro"] < gyro_floor:
        return "NONE"
    if f["peakGyro"] >= flick_gyro and f["ratio"] > ratio_cut:   # strict >, as firmware
        return "FLICK"
    return stage2(f, tree)


# ---- daily-HID action table incl. twist<->flick remap (ROADMAP #10) -----------
# Mirrors lyra_daily_hid.ino act()/servicePendingFlick(): the media action an
# ARMED gesture fires. `remap` swaps which motion carries VOLUME vs TRACK-SKIP so
# an anatomy-limited wearer gets volume on the flick they can make. Double-flick
# (arm/disarm) is handled separately and is not modeled here. Returns the media
# action string, or None for an unmapped case.
def hid_action(gesture, flick_right, twist_cw, remap=False):
    if gesture == "PINCH":
        return "play/pause"                      # unaffected by remap
    if gesture == "TWIST":
        if remap:                                 # remap: twist -> track skip
            return "next track" if twist_cw else "previous track"
        return "volume up" if twist_cw else "volume down"
    if gesture == "FLICK":                        # a single (non-double) flick
        if remap:                                 # remap: flick -> volume
            return "volume up" if flick_right else "volume down"
        return "next track" if flick_right else "previous track"
    return None


# ---- fast cascade (calibration/collect builds; produced `classified` col) -----
def classify_fast(f, gyro_floor=GYRO_FLOOR, ratio_cut=RATIO_CUT,
                  pinch_gate=0.40, pinch_twist_boundary=700.0):
    if f["peakGyro"] < gyro_floor:
        return "NONE"
    if f["peakGyro"] < pinch_twist_boundary:
        return "NONE" if f["accelStd"] < pinch_gate else "PINCH"
    return "FLICK" if f["ratio"] > ratio_cut else "TWIST"


# ---- custom-gesture template: matcher + NVS blob layout -----------------------
STD_FLOOR = 1e-3                       # firmware guard (newlyra_final.ino 289)
# struct GTemplate { char name[16]; char action; float mean[9]; float std[9];
#                    float thresh; };  natural alignment -> 3 pad bytes @17.
GTEMPLATE_FMT = "<16sc3x9f9ff"
GTEMPLATE_SIZE = struct.calcsize(GTEMPLATE_FMT)     # must be 96


def custom_distance(mean, std, feat):
    s = 0.0
    for j in range(9):
        sd = std[j] if std[j] > STD_FLOOR else STD_FLOOR
        z = (feat[j] - mean[j]) / sd
        s += z * z
    return math.sqrt(s / 9)


def match_custom(templates, feat):
    """templates: [(name, action, mean, std, thresh)]. Returns index or -1."""
    best, best_dist = -1, 1e9
    for i, (_n, _a, mean, std, thresh) in enumerate(templates):
        d = custom_distance(mean, std, feat)
        if d < thresh and d < best_dist:
            best_dist, best = d, i
    return best


def pack_gtemplate(name, action, mean, std, thresh):
    """Serialize a template exactly as the ESP32 lays it out in NVS."""
    raw_name = name.encode()[:15].ljust(16, b"\x00")
    return struct.pack(GTEMPLATE_FMT, raw_name, action.encode()[:1],
                       *mean, *std, thresh)


def unpack_gtemplate(blob):
    vals = struct.unpack(GTEMPLATE_FMT, blob)
    name = vals[0].split(b"\x00")[0].decode()
    action = vals[1].decode()
    mean = list(vals[2:11])
    std = list(vals[11:20])
    thresh = vals[20]
    return name, action, mean, std, thresh


# ---- versioned NVS record (schema-version framing; lyra_classifier.h N1 fix) --
# Each persisted per-template NVS record is a 1-byte schema version stamp followed
# by the raw 96-byte GTemplate blob (record = [schemaVersion][GTemplate]). The
# version frames the struct WITHOUT changing it, so GTEMPLATE_SIZE stays 96 and
# the ASCII G-command wire format (below) is untouched. loadCustomNVS rejects any
# record whose first byte != GTEMPLATE_SCHEMA_VERSION (stale/foreign layout).
GTEMPLATE_SCHEMA_VERSION = 1
GTEMPLATE_RECORD_SIZE = 1 + GTEMPLATE_SIZE          # must be 97


def pack_gtemplate_record(name, action, mean, std, thresh,
                          version=GTEMPLATE_SCHEMA_VERSION):
    """Serialize a template exactly as saveCustomNVS lays the record out in NVS."""
    return bytes([version]) + pack_gtemplate(name, action, mean, std, thresh)


def load_gtemplate_record(record):
    """Mirror loadCustomNVS's per-record check: return the unpacked template, or
    None if the record is the wrong length or carries the wrong schema version
    (rejected -> firmware falls back to defaults)."""
    if len(record) != GTEMPLATE_RECORD_SIZE or record[0] != GTEMPLATE_SCHEMA_VERSION:
        return None
    return unpack_gtemplate(record[1:])


# ---- G-command wire format (app -> calibration build write-back) --------------
def gtemplate_to_wire(name, action, mean, std, thresh):
    """The exact ASCII line the Flutter app writes to the RX characteristic."""
    safe = name.replace(",", " ").replace("\n", " ")[:15]
    nums = ",".join(f"{v:.3f}" for v in list(mean) + list(std) + [thresh])
    return f"G,{safe},{action},{nums}"


def who_am_i_ok(raw):
    """Mirror lyra_imu.h whoAmIOK(): genuine chip is 0x68, clones report
    0x70/71/72/98 and work fine -- only 0x00/0xFF means nothing answered."""
    return not (raw == 0x00 or raw == 0xFF)


def gtemplate_from_wire(line):
    """The exact parse the calibration firmware performs (handleConfigLine)."""
    tok = line.split(",")
    if tok[0] != "G" or len(tok) < 22:
        raise ValueError("not a valid G line")
    name = tok[1][:15]
    action = tok[2][:1] or "P"
    mean = [float(t) for t in tok[3:12]]
    std = [float(t) for t in tok[12:21]]
    thresh = float(tok[21])
    return name, action, mean, std, thresh
