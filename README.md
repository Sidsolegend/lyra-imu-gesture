# LYRA gesture recognition: data, code, and firmware

This repo has everything needed to reproduce the numbers, figures, and stage-2
decision tree in the paper *LYRA: An App-Free, On-Device Hybrid Classifier for
Wrist-IMU Gesture Recognition*. It is not the full LYRA project (the Flutter
app, the 3D-printed enclosure, the robot-arm bridge, etc. live elsewhere) --
just the parts a reviewer or reader would want to check.

## What's here

- `data/` -- the two gesture datasets. `lyra_multisubject.csv` is the
  original three-subject breadboard collection (subjects anonymized to
  A-D); `real_10subj.csv` is the ten-subject wrist-mounted collection.
- `research/` -- the analysis scripts. `lyra_ml.py` is a leave-one-subject-out
  harness (stdlib only, no numpy/pandas/sklearn) that reproduces Tables 1-2
  and emits the stage-2 tree as C++. `cross_hardware.py` reproduces Tables
  3-6, the cross-hardware and within-subject comparisons. `make_figures.py`
  regenerates Figures 1-3. `firmware_port.py` is a Python port of the
  deployed classifier, used to evaluate it against raw sensor features
  without needing the actual microcontroller.
- `firmware/` -- the deployed classifier as it runs on the device
  (`lyra_daily_hid.ino`), with the shared DSP/classifier/IMU logic split
  into `lib/*.h` so the sketch itself is just the event loop.
- `figures/` -- the regenerated figures plus a photo of the two hardware
  mountings referenced in the paper.
- `paper/` -- the manuscript.

## Reproducing the tables

```bash
python3 research/lyra_ml.py data/lyra_multisubject.csv       # Tables 1-2
python3 research/cross_hardware.py                            # Tables 3-6
python3 research/make_figures.py                              # Figures 1-3
```

No dependencies beyond Python 3 and matplotlib (only needed for the last
one). The stage-2 tree `lyra_ml.py` emits from the committed data matches
the tree actually flashed to the device -- see the paper's Reproducibility
note in section 5.2 for the one caveat (depth/min-leaf-size sensitivity in
one boundary split).

## Data

Both CSVs have the same 12 columns: `subject, label, peakGyro, ratio,
durationCount, gySigned, gzSigned, peakAccel, accelMean, accelStd,
spikeCount, classified`. `label` is the ground-truth gesture, `classified`
is whatever the device's on-board rule cascade called it at collection
time -- that's the "no calibration" accuracy in Table 1.

Subject identifiers are single letters (A-D in the first collection, A-J in
the second) rather than names. Three participants in the second collection
are the same people as A/B/C in the first, which is what section 5.4's
within-subject comparison uses.

## Firmware

`lyra_daily_hid.ino` is Arduino/ESP32 code for an ESP32-C3 with an MPU-6050
or compatible clone on I2C (SDA=GPIO6, SCL=GPIO7). It reads the IMU,
extracts the same nine features the analysis scripts use, runs the same
two-stage classifier, and reports gestures over BLE as HID media keys, so
it works with any host without a companion app.

## License

MIT. See `LICENSE`.
