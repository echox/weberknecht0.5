$fn = 180;

difference() {
union() {
    translate([0,0,-2]) cube([57,20,2]);
    translate([0,8,0]) cube([55,4,10]);
    translate([48,8,0]) cube([9,4,15]);
    translate([48,8,0]) cube([8,4,15]);

}
  translate([2.5,8.75,0]) cube([45,2.5,120]);
  translate([14,10,10.25]) rotate([90,0,0]) cylinder(d=16.5,h=20);
  translate([36.5,10,10.25]) rotate([90,0,0]) cylinder(d=16.5,h=20);
  translate([52.5,20,15]) rotate([90,0,0]) cylinder(d=7,h=20);
    translate([7.5,10,4]) cube([35,2.5,120]);



}