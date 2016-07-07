$fn=180;
thickness=2;
length=50;
border=2;
servoWidth=13;
servoLength=23;

w=servoWidth+(2*border);
l=servoLength+(2*border);
cutOffset=10;

module foot() {
 
union() {
    difference() {
        cube([w,l,thickness]);
        translate([2,2,0]) cube([servoWidth,servoLength,thickness+cutOffset]);
    }
    
    translate([(w-3)/2,l,0]) cube([3,length,thickness]);
    
    // servo nibble
    translate([w/2,border-2,0]) cylinder(d=2,h=thickness+2);
    translate([w/2,servoLength+border+2,thickness]) cylinder(d=2,h=2);
}

}

foot();