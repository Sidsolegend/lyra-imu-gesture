// =====================================================================
//  LYRA — wrist enclosure  ·  PRO / product finish
//  Sealed 2-story body: MPU-6050 (lower) + ESP32-C3 SuperMini (upper),
//  rounded corners, pillowed top, chamfered base, recessed seam reveal,
//  pill USB-C port, debossed wordmark, integrated band lugs.
//  Optional clip-on battery sled (print separately, snaps on later).
//
//  part = "base" | "top" | "battery" | "plate" | "all" | "batt_on"
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
batt_L = 22;   batt_W = 17;   batt_H = 4.0;   // your LiPo cell (e.g. ~402025)
batt_clr = 0.5;  sled_wall = 1.4;  sled_floor = 1.0;
sled_arm_h = 1.8;  sled_arm_w = 14;  sled_barb = 0.8;
det_z = 1.0;  det_h = 0.9;  det_d = 0.9;     // detent groove on the case short ends

/* [Assembly] */
lap = 2.0;  lip_t = 0.9;  snap = true;  pry = true;  pry_w = 6;

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
    for(sx=[-1,1]) translate([sx*out_L/2, 0, det_z]) cube([det_d*2, sled_arm_w+1, det_h], center=true);
}

// ---------------------------------------------------------------------
module base(){
    // batt_wire hole is cut AFTER band_lugs() is unioned in, otherwise the +Y
    // lug (which overlaps the hole's XY at Z[0,lug_h]) re-fills its top half.
    difference(){
        union(){
            difference(){
                body_base();
                translate([0,0,z_floor]) box(inner_L,inner_W, base_cav+eps, 1.2);
                translate([0,0,z_floor-mpu_recess]) box(mpu_L+2*fit, mpu_W+2*fit, mpu_recess+eps, 0.8);
                if(snap) for(sx=[-1,1]) translate([sx*inner_L/2,0,z_baseTop-1.2]) rotate([0,90,0]) cylinder(h=2,d=1.6,center=true);
                if(batt_mount) batt_detents();
            }
            if(band) band_lugs();
        }
        if(batt_wire)  translate([out_L/2-3.5, out_W/2-3.5, -eps]) cylinder(h=floor_t+1, d=3); // sled->cavity pass
    }
}

module top(){
    difference(){
        union(){
            body_top();
            translate([0,0,z_baseTop-lap])
                difference(){
                    box(inner_L-2*fit, inner_W-2*fit, lap+eps, 0.8);
                    translate([0,0,-eps]) box(inner_L-2*fit-2*lip_t, inner_W-2*fit-2*lip_t, lap+3*eps, 0.5);
                }
        }
        translate([0,0,z_divTop]) box(esp_L+2*fit, esp_W+2*fit, (z_topIn-z_divTop)+eps, 1.0);
        wirerow(z_baseTop-lap-eps, divider_t+lap+2*eps);
        usb_port();
        if(pry) translate([out_L/2,0,z_baseTop]) cube([wall*3, pry_w, 2.0], center=true);
        if(logo) translate([0,0,z_topOut-logo_d])
            linear_extrude(logo_d+eps) text(logo_txt, size=logo_sz, halign="center", valign="center",
                                            font="Liberation Sans:style=Bold");
    }
    if(snap) for(sx=[-1,1]) translate([sx*(inner_L-2*fit)/2,0,z_baseTop-1.2]) rotate([0,90,0]) cylinder(h=0.9,d=1.5,center=true);
}

// ---- battery sled (prints separately, snaps onto the base) ----------
module battery_sled(){
    pocket_h = batt_H + batt_clr;
    depth = sled_floor + pocket_h;                 // tray height; rim (= case bottom) at z=depth
    difference(){
        union(){
            box(out_L, out_W, depth, Rc);                                   // tray, flush with case
            for(sx=[-1,1]){                                                 // clip arms up the short ends
                translate([sx*(out_L/2 - sled_wall/2), 0, depth + sled_arm_h/2 - eps]) cube([sled_wall, sled_arm_w, sled_arm_h+2*eps], center=true);
                translate([sx*(out_L/2 - sled_wall - sled_barb/2 + 0.1), 0, depth + det_z - 0]) cube([sled_barb+0.2, sled_arm_w, det_h], center=true); // barb
            }
        }
        translate([0,0,sled_floor]) box(batt_L+batt_clr, batt_W+batt_clr, pocket_h+eps, 1.0); // cell pocket
        if(batt_wire) translate([out_L/2-3.5, out_W/2-3.5, sled_floor]) cylinder(h=pocket_h+1, d=3.2); // wire exit up
        translate([0, out_W/2, sled_floor+pocket_h/2]) cube([6, sled_wall*4, pocket_h], center=true);  // side wire/relief notch
    }
}

// ---------------------------------------------------------------------
module assembled(){ base(); if(mono) top(); else color("Gainsboro") top(); }

if      (part=="base")    base();
else if (part=="top")     translate([0,0,-(z_baseTop-lap)]) top();
else if (part=="battery") battery_sled();
else if (part=="plate"){ base(); translate([0, out_W+2*lug_ext+6, -(z_baseTop-lap)]) top(); translate([out_L+8,0,0]) battery_sled(); }
else if (part=="batt_on"){ assembled(); translate([0,0,-(sled_floor+batt_H+batt_clr)]) if(mono) battery_sled(); else color("DimGray") battery_sled(); }
else assembled();
