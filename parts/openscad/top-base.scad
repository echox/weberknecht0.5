module servo() {  
    
    L = 22.8;
    l = 12.6;
    h = 22.8;
    plateL = 32.5;
    plateH = 2.7;
    plateHPos = 16;
    topCylinderH = 3.4;
    smallTopCylinderD = 4.5;
    
    axisH = 2;
    axisD = 4;
    
    screwHoleCenter=2;
    screwHoleD=2;
    holeSize=1;
    
    difference() {
        union() {
            color("LightBlue", 0.5) {
                // main part
                cube([L, l, h]);
                // support
                translate([-(plateL - L) / 2, 0, plateHPos]) {
                    cube([plateL, l, plateH]);
                }
                // top big cylinder
                translate([l/2,l/2,h]) {
                    cylinder(d=l, h=topCylinderH, $fn=180);
                }
                // top small cylinder
                translate([l, l/2, h]) { 
                    cylinder(d=smallTopCylinderD, h=topCylinderH, $fn=180);
                }
            }
            translate([l/2,l/2, h + topCylinderH]) {
                color("white") {
                    cylinder(d=axisD,h=axisH, $fn=180);
                }
            }
        }
        translate([-(plateL - L) / 2 + screwHoleCenter, l/2, plateHPos]) {
            cylinder(d=screwHoleD, h=10, $fn=180, center=true);
        }
        translate([-(plateL - L) / 2 - 1, l / 2 - holeSize / 2, plateHPos - 1]) {
            cube([3,holeSize,4]);
        }
        translate([plateL - (plateL - L) / 2 - screwHoleCenter, l / 2, plateHPos - 1]) {
            cylinder(d=screwHoleD, h=10, $fn=180, center=true);
        }
        translate([plateL- (plateL - L) / 2 - screwHoleCenter, l / 2 - holeSize / 2, plateHPos -1]) {
            cube([3,holeSize,4]);
        }
    }
}





module servoArm(h) {
    linear_extrude(height = h) {
    circle(d=7.5+0.6,$fn=180);
    hull() {
    circle(d=6);
    translate([15,0,0]) circle(d=4+0.6,$fn=180);
    }
    }
}

module nible() {
    
union() {
//translate([0,0,4])cylinder(h=4, d=4.6, $fn=180);
cylinder(h=2,d=8, $fn=180);
cylinder(h=4,d=4, $fn=180);
}   
}


rotate([0,-90,0]) {
difference() {
    // base cube
    translate([0,0.2,-2])cube([20,32,18]);
    
    // additional spacer
    translate([2,4.5,2]) cube([20,24,20]);

    // top cut
    translate([0,0,14.5]) cube([20,33,5]);

    // servo inlet
    translate([2,5,2]) rotate([90,0,90]) servo();
    
    // cut side
    translate([18,0,-2]) cube([6,33,7]);

    // nible
   translate ([9,14,-2]) rotate([0,0,90]) servoArm(2);    
    
    // cable hole
    translate([4,0,3]) cube([5,5,9]);
    
    // cable slit
    translate([4,3.5,6]) cube([15,2.5,5]);
    translate([4,3,6]) cube([10,2.5,5]);
    
    // rotation socket
    translate ([0,11,8]) rotate([0,90,0]) cylinder(h=40,d=8,$fn=180);
    
    //cut bottom
    translate([-28,0,-5]) cube([30,40,30]);
    
    // servo screw
    translate([9,14,-5]) cylinder(d=2,h=20,$fn=180);    
    # translate([9,14,1]) cylinder(d=4,h=20,$fn=180);
}
}




