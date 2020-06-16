$fn=100;

difference() {
    union() {
        rotate([75,0,0]) translate([0,90/2-1,0]) 
        difference() {
            cube([190,90,3], center=true);                                              // Base Plate
            translate([-139/2-10,40,0]) {                                               // Display-Holes
                translate([  0,    0,0])    cylinder(h=50,d=3,center=true);
                translate([  0,-55.5,0])    cylinder(h=50,d=3,center=true);
                translate([139,    0,0])    cylinder(h=50,d=3,center=true);
                translate([139,-55.5,0])    cylinder(h=50,d=3,center=true);
            }
            translate([-10,5,0]) cube([60,30,10],center=true);                          // Center Hole
            translate([-10,90/2,0]) cube([120,20,20],center=true);                      // Display connectors
            translate([-63,40,0]) cube([20,30,10],center=true);                         // Opening for poti
            translate([55-10,-20,0]) union() {                                          // Arduino-Holes (Mega)
                translate([-13.97, 2.54, 0]) cylinder(h=20,d=3.5,center=true);           
                translate([-15.24, 50.8, 0]) cylinder(h=20,d=3.5,center=true);           
                translate([-96.52, 2.54, 0]) cylinder(h=20,d=3.5,center=true);           
                translate([-90.17, 50.8 ,0]) cylinder(h=20,d=3.5,center=true);           
            }
            translate([80,20,0]) cylinder(h=50,d=7,center=true);                         // Encoder Hole (Axis)
            translate([80,30,0]) cylinder(h=50,d=2,center=true);                        // Encoder Hole (Mouting)
        }

        // Sides
        difference() {
            union() {
                translate([190/2-1.5,35,85/2]) cube([3,70,85],center=true);
                translate([-190/2+1.5,35,85/2]) cube([3,70,85],center=true);
            }
            rotate([75,0,0]) translate([0,90/2,20]) cube([200,90,40], center=true);
            translate([0,70,80]) rotate([-45,0,0]) cube([200,90,40], center=true);
            translate([50,25,5]) rotate([0,90,0]) cylinder(h=100,d=3,center=true);       // Dust Sensor Hole 1
            translate([50,25+42,5]) rotate([0,90,0]) cylinder(h=100,d=3,center=true);    // Dust Sensor Hole 2

            translate([50,25+21,25]) rotate([0,90,0]) cylinder(h=100,d=10,center=true);   // Dust Sensor Ventilation
            
        }
    }
    
    translate([0,0,-5]) cube([200,200,10],center=true);                                   // Ground plane
}

