include <./lib/gears.scad>;

$fn = 128;


// ---- params you set ----
sensor_spacing = 2.135;   // mm, your PCB c-to-c
tab_count      = 10;      // number of white tabs around
disk_height    = 3;
pocket_offset  = 0.5;
pocket_module  = 0.23;
pocket_teeth   = 10;
pocket_height  = 3;
tab_tan_width  = 3;
tab_rad_depth  = 1.5;
tab_height     = 0.4;
// ------------------------

// compute disk radius so that 1/4 pitch == sensor_spacing
// pitch = 2*pi*R / tab_count
// we want pitch/4 = sensor_spacing  ->  R = sensor_spacing * 4 * tab_count / (2*pi)
disk_radius = sensor_spacing * 4 * tab_count / (2 * PI);
sense_radius = disk_radius;

union() {
  // base disk + center pocket
  difference() {
    cylinder(h = disk_height, r = disk_radius);
    translate([0,0,pocket_offset])
      spur_gear(pocket_module, pocket_teeth, pocket_height, 0);
  }

  // optical tabs
  for (i = [0:tab_count-1]) {
    rotate([0,0,360/tab_count * i])
      translate([sense_radius - tab_rad_depth, -tab_tan_width/2, disk_height])
        cube([tab_rad_depth, tab_tan_width, tab_height]);
  }
}
