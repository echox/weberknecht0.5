$fn=180;
length=100;
legHeight=1;

servoLength=20;

//servo holder
cube([2,10,10]);
translate([20,0,0]) cube([2,10,10]);
//servo base
cube([servoLength,10,2]);

linear_extrude(height=legHeight) {
hull(){
    translate([20,5,0]) circle(d=10);
    translate([30,5,0]) circle(d=5);
}

hull() {
    translate([20,5,0]) circle(d=5);
    translate([length,5,0]) circle(d=5);
}
}

translate([length,5,legHeight]) cylinder(d=4.8,h=1.5);
