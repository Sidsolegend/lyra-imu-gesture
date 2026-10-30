// =====================================================================
//  LYRA — wrist enclosure  ·  v2 (verified)
//  Same product language as the PRO case (rounded corners, pillowed top,
//  chamfered base, recessed seam reveal, pill USB-C port, deboss, lugs)
//  with two mechanical fixes found by verification of the PRO files:
//   1. SNAP FIX — PRO bumps engaged the sockets by only ~0.05mm (all of
//      the 0.45mm protrusion was eaten by the 0.4mm fit gap). v2 bumps
//      protrude gap + snap_eng and the sockets are widened to match.
//   2. SLED FIX — PRO clip arms sat INSIDE the case-wall footprint (the
//      sled could not physically mount). v2 arms wrap OUTSIDE the short
//      ends with inward barbs into raised detent grooves, and the tray
//      is lengthened to carry them.
//  Also: chamfered USB-C entry flare, and fitcheck parts that must
//  render EMPTY (see fc_* at the bottom).
//
//  part = "base" | "top" | "battery" | "plate" | "all" | "batt_on"
//       | "fc_shells" | "fc_boards" | "fc_sled"
// =====================================================================

part = "all";
mono = false;     // true = preview in one colour (how it really prints)

/* [ESP32-C3 SuperMini] */
esp_L = 22.5;  esp_W = 18.0;  esp_H = 1.4;
esp_clear = 3.4;          // headroom above PCB for USB-C body / antenna (tallest part)

/* [MPU-6050 / GY-521] */
mpu_L = 19.0;  mpu_W = 15.0;  mpu_H = 3.0;  mpu_recess = 0.8;

/* [Shell] */
wall = 1.4;  floor_t = 1.4;  divider_t = 1.0;  top_face_t = 1.0;
fit = 0.4;   mpu_squeeze = 0.2;

/* [Product styling] */
Rc      = 3.2;   // vertical corner radius (the soft corners)
Ftop    = 2.2;   // top-edge fillet (pillow)
bot_ch  = 1.0;   // bottom-edge chamfer (printable, premium)
seam_ch = 0.5;   // recessed reveal at the parting line
logo    = true;  logo_txt = "LYRA"; logo_sz = 5.5; logo_d = 0.6;

/* [USB-C port] */
usb_pw = 9.2;   usb_ph = 3.4;   usb_cz = 1.7;

/* [Inter-floor wiring] */
wire_d = 2.4;   wire_per = 3;

/* [Band lugs] */
band = true;  band_w = 14;  band_t = 2.4;  lug_ext = 4.5;  lug_h = 3.2;

/* [Battery sled — optional clip-on, printed separately] */
batt_mount = true;   // tiny detents on the short ends so the sled clicks on
batt_wire  = true;   // small wire pass from the sled up into the case
batt_L = 25;   batt_W = 16;   batt_H = 5.0;   // your LiPo cell, measured (~25x16x5, JST-PH lead off one short end)
batt_clr = 0.5;  sled_wall = 1.4;  sled_floor = 1.0;
sled_arm_w = 14;
det_z = 2.6;  det_h = 0.9;  det_d = 0.9;     // detent groove on the case short ends (below the seam band)
arm_clr = 0.15;   // flex gap between arm inner face and the case end wall
barb_eng = 0.6;   // barb reach inward from the arm face (hook depth = barb_eng - arm_clr)
sled_lip = 0.8;   // Y-registration lip on the tray rim

/* [Assembly] */
lap = 2.0;  lip_t = 0.9;  snap = true;  pry = true;  pry_w = 6;
snap_eng = 0.35;   // how far the bump reaches PAST the cavity wall face (real engagement)
snap_bd  = 1.6;    // bump diameter
snap_sd  = 1.9;    // socket diameter (bump + clearance)

$fn = 64;  eps = 0.01;

// ---- footprint / z-levels ------------------------------------------
board_L = max(esp_L, mpu_L);  board_W = max(esp_W, mpu_W);
inner_L = board_L + 2*fit;    inner_W = board_W + 2*fit;
out_L   = inner_L + 2*wall;   out_W   = inner_W + 2*wall;

base_cav  = (mpu_H - mpu_recess) + mpu_squeeze;
z_floor   = floor_t;
z_baseTop = z_floor + base_cav;
z_divTop  = z_baseTop + divider_t;
z_espTop  = z_divTop + esp_H;
z_topIn   = z_espTop + esp_clear;
z_topOut  = z_topIn + top_face_t;

module rrect(L,W,r){ hull() for(sx=[-1,1],sy=[-1,1]) translate([sx*(L/2-r),sy*(W/2-r)]) circle(max(r,0.3)); }
module box(L,W,h,r){ linear_extrude(h) rrect(L,W,r); }
module slice(L,W,r,z){ translate([0,0,z]) linear_extrude(eps) rrect(L,W,r); }

// ---- styled outer solids -------------------------------------------
module body_base(){
    hull(){
        slice(out_L-2*bot_ch, out_W-2*bot_ch, max(Rc-bot_ch,0.4), 0);
        slice(out_L, out_W, Rc, bot_ch);
        slice(out_L, out_W, Rc, z_baseTop-seam_ch);
        slice(out_L-2*seam_ch, out_W-2*seam_ch, max(Rc-seam_ch,0.4), z_baseTop);
    }
}
module body_top(){
    hull(){
        slice(out_L-2*seam_ch, out_W-2*seam_ch, max(Rc-seam_ch,0.4), z_baseTop);
        slice(out_L, out_W, Rc, z_baseTop+seam_ch);
        slice(out_L, out_W, Rc, z_topOut-Ftop);
        for(i=[0:8]) let(a=i*90/8, k=Ftop*(1-cos(a)))
            slice(out_L-2*k, out_W-2*k, max(Rc-k,0.4), z_topOut-Ftop+Ftop*sin(a));
    }
}

// ---- shared internal features --------------------------------------
wy = esp_W/2 + fit - wire_d/2 - 0.9;
module wirerow(z0,h){
    sp = (esp_L-7)/(wire_per-1);
    for(sy=[-1,1], i=[0:wire_per-1]) translate([-(esp_L-7)/2+i*sp, sy*wy, z0]) cylinder(h=h,d=wire_d);
}
module usb_port(){
    translate([-out_L/2, 0, z_espTop+usb_cz])
        hull() for(s=[-1,1]) translate([0,s*(usb_pw/2-usb_ph/2),0]) rotate([0,90,0]) cylinder(h=wall*4,d=usb_ph,center=true);
}
module band_lugs(){
    for(sy=[-1,1]) difference(){
        hull(){
            translate([0, sy*(out_W/2-Rc), lug_h/2]) cube([band_w+5, eps, lug_h], center=true);
            for(sx=[-1,1]) translate([sx*(band_w/2+1.5), sy*(out_W/2+lug_ext-1.5), lug_h/2]) cylinder(h=lug_h,d=3,center=true);
        }
        translate([0, sy*(out_W/2+lug_ext-band_t/2-1.2), -eps]) linear_extrude(lug_h+1) rrect(band_w, band_t, band_t/2);
    }
}
// detent grooves on the short ends (receive the sled barbs)
module batt_detents(){
    // groove width = barb width + 1 -> +/-0.5mm Y registration of the sled
    for(sx=[-1,1]) translate([sx*out_L/2, 0, det_z]) cube([det_d*2, (sled_arm_w-2)+1, det_h], center=true);
}

// ---------------------------------------------------------------------
module base(){
    // ALL interior cuts come after the lug union: the PRO version unioned
    // band_lugs() onto an already-hollowed body, so the lug hulls (anchored at
    // y=out_W/2-Rc, INSIDE the cavity extent) re-filled ~1mm of the cavity on
    // both Y sides — the lid skirt could not seat. Cutting the cavity last
    // carves the lug intrusion off flush with the inner wall.
    difference(){
        union(){ body_base(); if(band) band_lugs(); }
        translate([0,0,z_floor]) box(inner_L,inner_W, base_cav+eps, 1.2);
        translate([0,0,z_floor-mpu_recess]) box(mpu_L+2*fit, mpu_W+2*fit, mpu_recess+eps, 0.8);
        if(snap) for(sx=[-1,1]) translate([sx*inner_L/2,0,z_baseTop-1.2]) rotate([0,90,0]) cylinder(h=2.4,d=snap_sd,center=true);
        if(batt_mount) batt_detents();
        if(batt_wire)  translate([out_L/2-3.5, out_W/2-3.5, -eps]) cylinder(h=floor_t+1, d=3); // sled->cavity pass
    }
}

// ---- TOP: two pieces (v2.1) ------------------------------------------
// The v2 one-piece top was SEALED: the ESP pocket was walled by the divider
// below and the dome above, so the board could never be inserted (only its
// clearance was fit-checked, never an insertion path). v2.1 splits the top at
// the board's top face: the lower tower keeps the seam skirt/snaps/divider
// and an OPEN pocket the board drops into; a crown CAP (pillow + logo) snaps
// onto a tenon ring, watch-caseback style. The USB pill lives in the cap and
// notches the tenon — the connector nests in the notch as the cap slides on.
z_split = z_espTop;          // split plane = board top face
ten_t   = 0.6;               // cap wall thickness at the rebate
ten_h   = 1.2;               // tenon ring height
cap_clr = 0.1;               // sliding clearance, tenon vs cap rebate
module usb_flare(){          // chamfered entry flare on the port
    hull(){
        translate([-out_L/2-eps, 0, z_espTop+usb_cz])
            hull() for(s=[-1,1]) translate([0,s*(usb_pw/2-usb_ph/2),0]) rotate([0,90,0]) cylinder(h=eps,d=usb_ph+1.2);
        translate([-out_L/2+0.9, 0, z_espTop+usb_cz])
            hull() for(s=[-1,1]) translate([0,s*(usb_pw/2-usb_ph/2),0]) rotate([0,90,0]) cylinder(h=eps,d=usb_ph);
    }
}
module top_lower(){
    tw = out_W-2*(ten_t+cap_clr);        // tenon outer width (cap-snap bumps ride on it)
    difference(){
        union(){
            intersection(){ body_top(); translate([0,0,(z_baseTop+z_split)/2-eps]) cube([out_L+2,out_W+2,z_split-z_baseTop+2*eps],center=true); }
            translate([0,0,z_baseTop-lap])
                difference(){
                    box(inner_L-2*fit, inner_W-2*fit, lap+eps, 0.8);
                    translate([0,0,-eps]) box(inner_L-2*fit-2*lip_t, inner_W-2*fit-2*lip_t, lap+3*eps, 0.5);
                }
            translate([0,0,z_split-eps])                     // tenon ring
                difference(){
                    box(out_L-2*(ten_t+cap_clr), tw, ten_h+eps, max(Rc-ten_t-cap_clr,0.5));
                    translate([0,0,-eps]) box(esp_L+2*fit, esp_W+2*fit, ten_h+3*eps, 1.0);
                }
        }
        translate([0,0,z_divTop]) box(esp_L+2*fit, esp_W+2*fit, (z_topIn-z_divTop)+eps, 1.0);  // OPEN pocket: board drops in
        wirerow(z_baseTop-lap-eps, divider_t+lap+2*eps);
        usb_port(); usb_flare();                             // pill notches the tenon at -X
        if(pry) translate([out_L/2,0,z_baseTop]) cube([wall*3, pry_w, 2.0], center=true);
    }
    // seam snap bumps: from 0.5 inside the skirt face across the fit gap to
    // snap_eng PAST the cavity wall face — the PRO version stopped 0.05mm past.
    if(snap) for(sx=[-1,1]) let(x0=(inner_L-2*fit)/2-0.5, x1=inner_L/2+snap_eng)
        translate([sx*(x0+x1)/2,0,z_baseTop-1.2]) rotate([0,90,0]) cylinder(h=x1-x0,d=snap_bd,center=true);
    // cap snap bumps on the tenon's long faces (0.15 engagement past cap_clr)
    if(snap) for(sy=[-1,1]) let(y0=tw/2-0.5, y1=tw/2+0.25)
        translate([0,sy*(y0+y1)/2,z_split+0.55]) rotate([90,0,0]) cylinder(h=y1-y0,d=1.1,center=true);
}
module top_cap(){
    difference(){
        intersection(){ body_top(); translate([0,0,z_split+10]) cube([out_L+2,out_W+2,20],center=true); }
        translate([0,0,z_split-eps]) box(esp_L+2*fit, esp_W+2*fit, (z_topIn-z_split)+eps, 1.0);   // USB/antenna headroom
        translate([0,0,z_split-eps]) box(out_L-2*ten_t, out_W-2*ten_t, ten_h+cap_clr+eps, max(Rc-ten_t,0.5));  // rebate over the tenon
        usb_port(); usb_flare();
        if(logo) translate([0,0,z_topOut-logo_d])
            linear_extrude(logo_d+eps) text(logo_txt, size=logo_sz, halign="center", valign="center",
                                            font="Liberation Sans:style=Bold");
        // sockets for the tenon bumps
        if(snap) for(sy=[-1,1]) translate([0,sy*(out_W/2-ten_t-cap_clr+0.25),z_split+0.55]) rotate([90,0,0]) cylinder(h=0.9,d=1.5,center=true);
    }
}
module top(){ top_lower(); top_cap(); }   // assembled top (fitchecks/views)

// ---- battery sled (prints separately, snaps onto the base) ----------
// v2: clip arms wrap OUTSIDE the case end walls (PRO's arms occupied the
// same space as the walls), barbs hook inward into the raised detent
// grooves, tray lengthened/widened to carry the arms + a Y-registration lip.
sled_pocket = batt_H + batt_clr;
sled_depth  = sled_floor + sled_pocket;            // rim (= case bottom) at z=sled_depth
tray_L = out_L + 2*(arm_clr + sled_wall);
tray_W = out_W;                                    // flush with the case sides (lugs live above)
module battery_sled(){
    arm_top = sled_depth + det_z + det_h + 0.5;
    difference(){
        union(){
            hull(){                                                          // tray, chamfered like the case
                slice(tray_L-2*bot_ch, tray_W-2*bot_ch, max(Rc-bot_ch,0.4), 0);
                slice(tray_L, tray_W, Rc, bot_ch);
                slice(tray_L, tray_W, Rc, sled_depth);
            }
            for(sx=[-1,1]){                                                  // clip arms OUTSIDE the case ends
                translate([sx*(out_L/2+arm_clr+sled_wall/2), 0, arm_top/2]) cube([sled_wall, sled_arm_w, arm_top], center=true);
                translate([sx*(out_L/2+arm_clr-barb_eng/2+0.1), 0, sled_depth+det_z]) cube([barb_eng+0.2, sled_arm_w-2, det_h-0.2], center=true); // barb -> detent groove
            }
        }
        translate([0,0,sled_floor]) box(batt_L+batt_clr, batt_W+batt_clr, sled_pocket+eps, 1.0); // cell pocket
        if(batt_wire) translate([out_L/2-3.5, out_W/2-3.5, sled_floor]) cylinder(h=sled_pocket+1, d=3.2); // wire exit up
        translate([0, tray_W/2, sled_floor+sled_pocket/2]) cube([6, sled_wall*4, sled_pocket], center=true);  // side wire/relief notch
    }
}

// ---- fit checks: each must render EMPTY ------------------------------
module ghost_esp(){ translate([0,0,z_divTop]) box(esp_L, esp_W, esp_H, 0.8); }
module ghost_mpu(){ translate([0,0,z_floor-mpu_recess]) box(mpu_L, mpu_W, mpu_H, 0.8); }
module sled_on(){ translate([0,0,-sled_depth]) battery_sled(); }
module fc_shells(){ intersection(){ base(); top(); } }                       // lid on base (incl. snap bumps in sockets)
module fc_boards(){ intersection(){ union(){ base(); top(); } union(){ ghost_esp(); ghost_mpu(); } } }
module fc_sled(){ intersection(){ base(); sled_on(); } }                     // sled clipped on (arms/barbs vs walls/grooves)

// ---- built-in break-away support for the top (print aid, snaps off) --
// The top prints skirt-down and its seam underside (a 0.9mm-wide ring from
// the cavity wall out to the body face) appears at z=lap with NOTHING under
// it — the skirt outer face is 0.8mm inboard of it. Two real prints failed
// exactly there (perimeters curl, nozzle strikes the curl, part blobs out).
// This sacrificial ring stands on the bed directly under that band, stopping
// ring_gap short of it: the seam layer prints onto the ring but only tacks
// to it, and the ring snaps off at the (hidden) seam face after printing.
ring_gap = 0.2;    // vertical break gap (one layer at 0.2mm)
module seam_support(){
    difference(){
        box(out_L-0.8, out_W-0.8, lap-ring_gap, 2.5);   // under the seam band, clear of the flare
        // (corner r=2.5 < the body's r=2.7 seam edge so the ring tracks under the corner arcs)
        translate([0,0,-eps]) box(out_L-2.3, out_W-2.3, lap, max(Rc-0.75,0.3));  // 0.75mm wall; 0.65mm clear of the skirt
        // windows around the two snap bumps (they protrude to out_L/2-0.05,
        // 0.1mm INTO the ring wall — without these the ring fuses to the
        // bumps and snapping it off would shear them)
        for(sx=[-1,1]) translate([sx*(out_L/2-0.8), 0, lap/2]) cube([3.5, 7, lap+1], center=true);
    }
}
module top_print(){ translate([0,0,-(z_baseTop-lap)]) top_lower(); seam_support(); }
module cap_print(){ translate([0,0,-z_split]) top_cap(); }   // rim down, dome up

// ---------------------------------------------------------------------
module assembled(){ base(); if(mono) top(); else color("Gainsboro") top(); }

if      (part=="base")    base();
else if (part=="top")     top_print();
else if (part=="cap")     cap_print();
else if (part=="battery") battery_sled();
else if (part=="plate"){ base(); translate([0, out_W+2*lug_ext+6, 0]) top_print(); translate([0, -(out_W+2*lug_ext+6), 0]) cap_print(); translate([out_L+tray_L/2-out_L/2+10,0,0]) battery_sled(); }
else if (part=="batt_on"){ assembled(); if(mono) sled_on(); else color("DimGray") sled_on(); }
else if (part=="fc_shells") fc_shells();
else if (part=="fc_boards") fc_boards();
else if (part=="fc_sled")   fc_sled();
else assembled();
