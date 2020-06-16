$fn=100;

difference() {
    union() {
        rotate([75,0,0]) translate([0,90/2-1,0]) 
        difference() {
            cube([170,90,5], center=true);                                      // Base Plate
            translate([-139/2,40,0]) {                                          // Display-Holes
                translate([  0,    0,0])    cylinder(h=50,d=3,center=true);
                translate([  0,-55.5,0])    cylinder(h=50,d=3,center=true);
                translate([139,    0,0])    cylinder(h=50,d=3,center=true);
                translate([139,-55.5,0]) cylinder(h=50,d=3,center=true);
            }
            translate([0,5,0]) cube([60,30,10],center=true);                    // Center Hole
            translate([0,90/2,0]) cube([120,20,20],center=true);                // Display connectors
            translate([55,-20,0]) union() {                                     // Arduino-Holes (Mega)
                translate([-13.97, 2.54, 0]) cylinder(h=20,d=3.5,center=true);           
                translate([-15.24, 50.8, 0]) cylinder(h=20,d=3.5,center=true);           
                translate([-96.52, 2.54, 0]) cylinder(h=20,d=3.5,center=true);           
                translate([-90.17, 50.8 ,0]) cylinder(h=20,d=3.5,center=true);           
            }
            
        }

        // Sides
        difference() {
            union() {
                translate([170/2-2.5,35,25]) cube([5,70,50],center=true);
                translate([-170/2+2.5,35,25]) cube([5,70,50],center=true);
            }
            translate([0,70,50]) rotate([0,90,0]) scale([1,1.4,1]) cylinder(h=200,d=80,center=true);
            rotate([75,0,0]) translate([0,90/2,10]) cube([180,90,20], center=true);
            translate([0,20,15]) rotate([0,90,0]) cylinder(h=200,d=15,center=true);
            translate([0,25,5]) rotate([0,90,0]) cylinder(h=200,d=2,center=true);       // Dust Sensor Hole 1
            translate([0,25+42,5]) rotate([0,90,0]) cylinder(h=200,d=2,center=true);    // Dust Sensor Hole 2
        }
    }
    
    translate([0,0,-5]) cube([200,200,10],center=true); // Ground plane
}
