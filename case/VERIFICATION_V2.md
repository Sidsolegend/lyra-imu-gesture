# LYRA case v2 — verification report (2026-07-03)

`lyra_case_v2.scad` supersedes `lyra_case_pro.scad`. Same product styling
(pillowed top, seam reveal, chamfered base, USB-C pill, LYRA deboss, band
lugs), with three mechanical defects found in the PRO geometry and fixed here.

## Defects found in PRO (also present in the exported `stl/lyra_pro_*.stl`)

1. **Lid could not seat.** `band_lugs()` was unioned onto the already-hollowed
   body, and the lug hulls anchor at `y = out_W/2 - Rc` (7.6 mm) — *inside* the
   cavity extent (9.4 mm). ~48 mm³ of lug material re-filled the cavity on both
   Y sides, exactly where the lid skirt lands (~1 mm interference). It also
   missed the MPU-6050 by only 0.1 mm. **Fix:** all interior cuts now come
   after the lug union, carving the intrusion flush with the inner wall.
2. **Snap fit had no engagement.** Bumps protruded 0.45 mm but the fit gap ate
   0.40 mm, leaving ~0.05 mm in the sockets — friction only. **Fix:** bumps now
   reach `snap_eng = 0.35 mm` past the cavity wall face; sockets widened to
   1.9 mm for a 1.6 mm bump.
3. **Battery sled could not mount.** Clip arms sat *inside* the case-wall
   footprint (same space as the wall) and the barbs never reached the detent
   grooves. **Fix:** arms wrap outside the short ends (0.15 mm flex gap),
   barbs hook 0.45 mm into the grooves, tray lengthened to carry the arms,
   groove width tightened to give ±0.5 mm Y registration.

Also added: chamfered USB-C entry flare.

## Verification method

Fit-check parts render the intersection of mating/occupying solids — each must
be empty (OpenSCAD: "Current top level object is empty") or measure ~0 mm³:

| Check | What it intersects | Result |
|---|---|---|
| `fc_shells` | base ∩ lid, assembled (incl. snap bumps in sockets) | 0.69 mm³ — a 10 µm sheet at the seam plane (eps-slice construction contact, not interference) |
| `fc_boards` | case ∩ ESP32-C3 + MPU-6050 envelopes in their seats | empty |
| `fc_sled` | base ∩ sled, clipped on (arms, barbs, grooves) | 1.14 mm³ — 10 µm rim-contact sheet |

All three exported STLs are manifold ("Simple: yes" in CGAL).

## Print settings (Bambu P1S, PLA, 0.4 nozzle)

| Part | Size (mm) | Orientation | Supports | Layer | Walls | Infill |
|---|---|---|---|---|---|---|
| `lyra_v2_base.stl` | 26 × 31 × 4 | as exported (flat) | none | 0.12–0.16 | 3 | 25% |
| `lyra_v2_top.stl` | 26 × 22 × 4.4 | as exported (skirt down, pocket up) | none — **break-away ring built in** + 3 mm brim (see below) | 0.12–0.16 | 3 | 15% |
| `lyra_v2_cap.stl` | 26 × 22 × 4.4 | as exported (rim down, dome up) | none + 3 mm brim (narrow rim) | 0.12–0.16 | 3 | 15% |
| `lyra_v2_battery.stl` | 29 × 22 × 9.5 | as exported (tray down) | none | 0.16 | 3 | 20% |

### v2.1 — the top is now TWO pieces (2026-07-03)

The v2 one-piece top was **sealed**: the ESP pocket was walled by the divider
below and the dome above, so the board could never be inserted — only its
clearance was fit-checked, never an insertion path. v2.1 splits the top at
the board's top face:

- **`lyra_v2_top.stl`** (tower) — seam skirt/snaps/divider + an **open pocket**
  the ESP32 drops into, with a 1.2 mm tenon ring around the rim.
- **`lyra_v2_cap.stl`** (crown) — the pillowed dome + LYRA deboss; its rebate
  slides over the tenon and two side bumps click into sockets. The USB pill
  spans both pieces (it notches the tenon), so the connector nests in the
  notch as the cap slides on.

**Assembly:** MPU in the base recess → wires up through the divider holes →
drop the ESP into the open pocket (USB-C nose into the pill notch) → solder →
snap the tower onto the base → click the cap on. All three fit-checks
re-verified after the split (`fc_boards` empty; `fc_shells`/`fc_sled` the
usual 10 µm contact films). Print verification: the tower lost the old
internal dome bridge entirely; the cap's only mid-air region is the enclosed
headroom ceiling (a normal ~19 mm bridge).

> ⚠️ **Field correction (2026-07-03):** an earlier revision of this table said
> the top needs no supports ("1.4 mm internal bridge ring is fine"). Real
> prints of the skirt-down construction failed identically every time: for
> the first 2 mm only the 0.9 mm skirt ring prints, then the entire shell
> cross-section appears at z = `lap` with the outer wall ring **fully
> unsupported in air** (the skirt outer face at 22.5 mm doesn't even touch the
> wall ring, which starts at 23.3 mm). The perimeters curl, the nozzle strikes
> the curl, and the part blobs out. The dome means this part can't be flipped,
> so `lyra_v2_top.stl` now ships with a **built-in break-away support ring**
> (`seam_support()` in the SCAD): two C-shaped 0.75 mm walls standing on the
> bed directly under the seam band, stopping 0.2 mm short of it, windowed
> around the snap bumps so removal can't shear them. After printing, snap the
> two ring halves off the seam face (hidden when assembled). Still use a brim.
> The v1 (functional) top has no dome and its STLs are exported pre-flipped
> instead — rim on the bed, no supports and nothing to snap off.

~5 g total, ≈1 h for all three. Fine layers (0.12) recommended — the snap
bumps, barbs and 0.6 mm deboss are near nozzle resolution. Print the top in a
contrasting colour for the two-tone product look. If the snap is too tight on
your printer's tolerances, reduce `snap_eng` to 0.25; too loose, raise to 0.45.
