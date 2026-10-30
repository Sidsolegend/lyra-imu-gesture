// =====================================================================
//  LYRA — 2-story wrist enclosure  (ESP32-C3 SuperMini + MPU-6050)
//  Lower floor : MPU-6050 (GY-521) in a floor recess
//  Upper floor : ESP32-C3 SuperMini, USB-C accessible
//  Splits at the divider: solder the inter-board wires through the
//  pass-throughs, then press the top half onto the base (lip + snap).
//
//  WORKFLOW
//    1. Caliper your boards -> edit the dimension blocks below.
//    2. part = "base" | "top" | "lid" | "plate" (print) | "all" (preview)
//    3. F6, then File > Export > STL.
// =====================================================================

part = "all";          // render selector

/* [ESP32-C3 SuperMini] */
esp_L      = 22.5;     // board length (long axis)
esp_W      = 18.0;     // board width
esp_H      = 1.4;      // PCB thickness
usb_w      = 12.0;     // (legacy) lid USB opening width
usb_pw     = 9.2;      // USB-C port pill width
usb_ph     = 3.4;      // USB-C port pill height
usb_cz     = 1.7;      // port centre above the PCB top

/* [MPU-6050 / GY-521 module] */
mpu_L      = 19.0;     // module length  (your 19 x 15 compartment)
mpu_W      = 15.0;     // module width
mpu_H      = 3.0;      // module thickness (direct-solder, no header)
mpu_recess = 0.8;      // how deep the module sinks into the floor

/* [Enclosure shell] */
wall       = 1.2;      // side walls (3 perimeters @ 0.4 nozzle)
floor_t    = 1.4;      // bottom floor
divider_t  = 1.0;      // inter-floor divider
fit        = 0.4;      // clearance around each board (per side)
esp_lip    = 2.0;      // wall height above the ESP32 PCB
mpu_squeeze= 0.2;      // gap above the MPU (divider lightly captures it)

/* [Inter-floor wiring] */
// Holes sit in a cluster along EACH long edge of the divider so the 4 nets
// reach the right ESP32 pads with short wires:
//   POWER edge  : MPU VCC -> ESP 3V3 ,  MPU GND -> ESP GND
//   I2C  edge   : MPU SCL -> ESP GPIO6 , MPU SDA -> ESP GPIO7
wire_d     = 2.4;      // pass-through diameter
wire_per   = 3;        // holes per long edge (need 2 each: power + I2C)

/* [Wrist strap] */
strap      = true;     // slots through the long walls
strap_w    = 13;       // slot length
strap_t    = 2.6;      // slot height (strap thickness)

/* [Assembly + lid] */
lap        = 2.0;      // alignment-lip depth into the base
lip_t      = 0.9;      // alignment-lip thickness
snap       = true;     // friction snap bump / dimple
pry        = true;     // thumbnail notch at the seam to lever the halves apart
pry_w      = 6;        // pry-notch width
make_lid   = false;    // clip cover over the ESP32 (part="lid")

/* [Battery sled — optional clip-on, printed separately, part="battery"] */
batt_mount = true;     // tiny detents on the short ends so the sled clicks on
batt_wire  = true;     // small wire pass from the sled up into the case
batt_L     = 22;       // LiPo cell length  (e.g. ~402025)
batt_W     = 17;       // LiPo cell width
batt_H     = 4.0;      // LiPo cell thickness
batt_clr   = 0.5;      // pocket clearance
sled_wall  = 1.2;
sled_floor = 1.0;
sled_arm_h = 1.8;      // clip-arm rise onto the case short ends
sled_arm_w = 14;       // clip-arm width
sled_barb  = 0.8;      // inward catch
det_z      = 1.0;  det_h = 0.9;  det_d = 0.9;   // detent groove on the short ends

$fn = 48; eps = 0.01;

// ---- footprint (driven by the larger board) -------------------------
board_L = max(esp_L, mpu_L);
board_W = max(esp_W, mpu_W);
inner_L = board_L + 2*fit;
inner_W = board_W + 2*fit;
out_L   = inner_L + 2*wall;
out_W   = inner_W + 2*wall;

// ---- z levels -------------------------------------------------------
mpu_stand = mpu_H - mpu_recess;
base_cav  = mpu_stand + mpu_squeeze;
z_floor   = floor_t;
z_baseTop = z_floor + base_cav;          // split line
z_divTop  = z_baseTop + divider_t;       // ESP rests here
z_espTop  = z_divTop + esp_H;
z_topWall = z_espTop + esp_lip;

module rrect(L,W,r){ hull() for(sx=[-1,1],sy=[-1,1]) translate([sx*(L/2-r),sy*(W/2-r)]) circle(r); }
module box(L,W,h,r=1.2){ linear_extrude(h) rrect(L,W,r); }

// wire clusters: one row inset from EACH long edge (power edge + I2C edge)
wy = esp_W/2 + fit - wire_d/2 - 0.8;     // inset from the long wall
module wirerow(z0,h){
    sp = (esp_L-7)/(wire_per-1);
    for(sy=[-1,1], i=[0:wire_per-1])
        translate([-(esp_L-7)/2 + i*sp, sy*wy, z0]) cylinder(h=h, d=wire_d);
}

// rounded (pill) USB-C cutout in the -X short wall
module usb_port(){
    translate([-out_L/2, 0, z_espTop+usb_cz])
        hull() for(s=[-1,1]) translate([0,s*(usb_pw/2-usb_ph/2),0]) rotate([0,90,0]) cylinder(h=wall*4,d=usb_ph,center=true);
}

// ---------------------------------------------------------------------
module base(){
    difference(){
        box(out_L,out_W,z_baseTop,r=1.6);
        // upper cavity (full inner footprint) above the floor
        translate([0,0,z_floor]) box(inner_L,inner_W,base_cav+eps, r=1.0);
        // MPU recess sunk into the floor
        translate([0,0,z_floor-mpu_recess]) box(mpu_L+2*fit, mpu_W+2*fit, mpu_recess+eps, r=0.8);
        // strap slots
        if(strap) for(sy=[-1,1]) translate([0,sy*out_W/2,floor_t/2+strap_t/2]) cube([strap_w,wall*4,strap_t],center=true);
        // snap dimples in the inner short walls
        if(snap) for(sx=[-1,1]) translate([sx*inner_L/2,0,z_baseTop-1.2]) rotate([0,90,0]) cylinder(h=2,d=1.6,center=true);
        // battery-sled detents on the short ends
        if(batt_mount) for(sx=[-1,1]) translate([sx*out_L/2,0,det_z]) cube([det_d*2, sled_arm_w+1, det_h], center=true);
        // battery wire pass (sled -> cavity), corner
        if(batt_wire) translate([out_L/2-3.5, out_W/2-3.5, -eps]) cylinder(h=floor_t+1, d=3);
        // (no other floor holes: wires drop from the divider straight into this cavity to the MPU)
    }
}

// ---- battery sled (prints separately, snaps onto the base) ----------
module battery_sled(){
    pocket_h = batt_H + batt_clr;
    depth = sled_floor + pocket_h;             // rim (= case bottom) at z=depth
    difference(){
        union(){
            box(out_L, out_W, depth, r=1.6);
            for(sx=[-1,1]){                                              // clip arms + barbs
                translate([sx*(out_L/2 - sled_wall/2), 0, depth + sled_arm_h/2 - eps]) cube([sled_wall, sled_arm_w, sled_arm_h+2*eps], center=true);
                translate([sx*(out_L/2 - sled_wall - sled_barb/2 + 0.1), 0, depth + det_z]) cube([sled_barb+0.2, sled_arm_w, det_h], center=true);
            }
        }
        translate([0,0,sled_floor]) box(batt_L+batt_clr, batt_W+batt_clr, pocket_h+eps, r=1.0);  // cell pocket
        if(batt_wire) translate([out_L/2-3.5, out_W/2-3.5, sled_floor]) cylinder(h=pocket_h+1, d=3.2);
        translate([0, out_W/2, sled_floor+pocket_h/2]) cube([6, sled_wall*4, pocket_h], center=true); // wire relief
    }
}

// ---------------------------------------------------------------------
module top(){
    difference(){
        union(){
            translate([0,0,z_baseTop]) box(out_L,out_W,z_topWall-z_baseTop,r=1.6);   // upper shell
            // alignment lip descending into the base
            translate([0,0,z_baseTop-lap])
                difference(){
                    box(inner_L-2*fit, inner_W-2*fit, lap+eps, r=0.8);
                    translate([0,0,-eps]) box(inner_L-2*fit-2*lip_t, inner_W-2*fit-2*lip_t, lap+3*eps, r=0.5);
                }
        }
        // ESP32 pocket
        translate([0,0,z_divTop]) box(esp_L+2*fit, esp_W+2*fit, (z_topWall-z_divTop)+eps, r=1.0);
        // wire pass-throughs through the divider
        wirerow(z_baseTop-lap-eps, divider_t+lap+2*eps);
        // rounded USB-C port in the -X short wall
        usb_port();
        // pry notch at the +X short end (over the snap) — gap at the seam
        if(pry) translate([out_L/2, 0, z_baseTop]) cube([wall*3, pry_w, 2.0], center=true);
    }
    // snap bumps on the lip outer faces
    if(snap) for(sx=[-1,1]) translate([sx*(inner_L-2*fit)/2, 0, z_baseTop-1.2]) rotate([0,90,0]) cylinder(h=0.9,d=1.5,center=true);
}

// ---------------------------------------------------------------------
module lid(){
    difference(){
        box(out_L,out_W,divider_t+2.0,r=1.6);
        translate([0,0,-eps]) box(inner_L-2*fit, inner_W-2*fit, 2.0, r=0.8);
        translate([-out_L/2,0,divider_t]) cube([wall*4,usb_w,6],center=true);  // keep USB end open
    }
}

// ---------------------------------------------------------------------
// print orientation for the top: FLIPPED (wall rim on the bed, lip up).
// Lip-down, the first 2 mm is only the 0.9 mm lip ring and the whole shell
// cross-section then appears in mid-air at z=lap — it curled and failed on
// two real prints. Flipped, every layer is supported; brim, no supports.
module top_print(){ translate([0,0,z_topWall]) rotate([180,0,0]) top(); }

if      (part=="base")    base();
else if (part=="top")     top_print();
else if (part=="lid")     lid();
else if (part=="battery") battery_sled();
else if (part=="plate"){ base(); translate([0,out_W+4,0]) top_print(); translate([out_L+8,0,0]) battery_sled(); if(make_lid) translate([0,2*(out_W+4),0]) lid(); }
else if (part=="batt_on"){ base(); color("SteelBlue") top(); color("DimGray") translate([0,0,-(sled_floor+batt_H+batt_clr)]) battery_sled(); }
else { base(); color("SteelBlue") top(); if(make_lid) color("Gainsboro") translate([0,0,z_topWall+1]) lid(); }
