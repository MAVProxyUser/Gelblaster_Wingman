// Parameters
belt_width = 9;           // Width of the timing belt
belt_thickness = 3;       // Combined thickness of two stacked belts
clearance = 0.8;          // Additional clearance for ease of use

// Derived dimensions
outer_width = belt_width + 2 * clearance;
inner_width = belt_width;
section_height = belt_thickness + clearance; // Height of each of the three sections
height = 3 * section_height;  // Total height

module tri_glide_buckle() {
    difference() {
        // Outer shape
        cube([outer_width, height, belt_thickness + clearance], center=true);

        // Inner cutouts
        translate([0, section_height/2, 0])
        cube([inner_width, section_height, belt_thickness + clearance], center=true);
        translate([0, -section_height/2, 0])
        cube([inner_width, section_height, belt_thickness + clearance], center=true);
    }

    // Central divider extended to touch the sides
    cube([belt_thickness-clearance, height, belt_thickness + clearance], center=true);
}

// Call the module to create the buckle
tri_glide_buckle();


