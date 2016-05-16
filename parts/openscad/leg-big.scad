$fn=180;
length=100;
legHeight=2;
shrinkDelta=0;

module servoArm(h) {
    linear_extrude(height = h) {
    circle(d=7.5+0.3);
    hull() {
    circle(d=6);
    translate([15,0,0]) circle(d=4+0.3);
    }
    }
}

module servoHole() {
    cylinder(d=12+shrinkDelta,h=legHeight);
    translate([6,0,0]) cylinder(d=6+shrinkDelta,h=legHeight);
}

module leg() {
union(){
linear_extrude(height=legHeight){
    hull() {
      circle(d=16);
      translate([12,0,0]) circle(d=5);
    }   
    hull() {
        translate([12,0,0]) circle(d=4);
        translate([length,0,0]) circle(d=4);
    }
    hull(){       
      translate([length,0,0]) circle(d=4);
      translate([length+10,0,0]) circle(d=14);
      translate([length+10+10,0,0]) circle(d=14);
    }
}
}
 //translate([length+10+10,0,legHeight]) cylinder(h=1.5,d=10);
 translate([length+20,0,1.5]) rotate([0,0,180]) scale(1.2)servoArm(2);
}


difference() {
    leg();
    translate([length+20,0,1.5]) rotate([0,0,180]) servoArm(10);
    servoHole();
}