$fn = 100;

// Parameters for the triangular column
side_length = 20.5;
radius = 4.25;
height = 75;

// Calculate height of the equilateral triangle
h = side_length * sqrt(3) / 2;

// Adjusted positions for circle centers
center1 = [0, h/2 - radius];
center2 = [-6, -h/2 + radius];
center3 = [6, -h/2 + radius];

// Parameters for the tabletop
table_thickness = 5;
hole_spacing = 36; // Adjusted hole spacing
edge_distance = 5;
screw_hole_diameter = 3;
table_side = hole_spacing + 2 * edge_distance;

// Parameters for rounded_triangle
total_length = 18.5; 
ellipse_protrusion = 2.5;
flat_side = total_length - ellipse_protrusion;

module rounded_triangle() {
    // Define the rectangle's left corners
    corner1 = [-flat_side/2, -11/2];
    corner2 = [-flat_side/2, 11/2];

    // Define the center point for the ellipse on the right side
    ellipse_center = [flat_side/2, 0];

    // Use hull() to create the shape with the ellipse for one side
    hull() {
        translate(corner1) circle(0.01); 
        translate(corner2) circle(0.01);  
        translate(ellipse_center + [ellipse_protrusion, 0]) scale([ellipse_protrusion, 11/2]) circle(1); 
    }
}

module tabletop(){
    difference() {
        cube([table_side, table_side, table_thickness], center=true);
        for (i = [-1, 1]) {
            for (j = [-1, 1]) {
                translate([(i * hole_spacing / 2), (j * hole_spacing / 2), table_thickness / 2])  
                cylinder(r=screw_hole_diameter/2, h=100, center=true);
            }
        }
    }
}

union() {
    linear_extrude(height)
    rounded_triangle();
    translate([0, 0, height]);
    tabletop();
}


