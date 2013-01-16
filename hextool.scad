
module hextool()
{
translate([0,0,0]){cylinder(5,21.5,23.8);};
translate([0,0,5]) {cylinder(7,23.8,5);};
translate([0,0,15]) {cube([35,6,20],true);}
rotate(90){translate([0,0,15]) {cube([35,6,20],true);}}
translate([23,0,0]) {cylinder(5,2,2);}
rotate(120){translate([23,0,0]) {cylinder(5,2,2);}}
rotate(240){translate([23,0,0]) {cylinder(5,2,2);}}
}

hextool();

