
module hextool()
{
cylinder(5,23,25);
translate([0,0,5]) {cylinder(7,25,5);};
translate([0,0,15]) {cube([35,6,20],true);}
rotate(90){translate([0,0,15]) {cube([35,6,20],true);}}
translate([24,0,0]) {cylinder(5,2,2);}
rotate(120){translate([24,0,0]) {cylinder(5,2,2);}}
rotate(240){translate([24,0,0]) {cylinder(5,2,2);}}
}

hextool();

