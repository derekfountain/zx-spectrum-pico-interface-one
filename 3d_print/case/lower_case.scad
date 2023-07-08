pcb_width=99.85;                // Across the back of the ZX, as per KiCAD PCB layout
pcb_length=99.96;               // ZX edge connector to rear of PCB, as per KiCAD PCB layout
side_wall_thickness=2.0;        // Left and right side walls
far_wall_thickness=1.0;         // Wall as rear of device, might obstruct plugging in
closest_wall_thickness=1.0;     // Wall closest to ZX, might obstruct plugging in
base_thickness=1.0;             // Mustn't be too high, PCB will lift as it is

walls_height=10.0;              // For now, keep the walls low

// These are from the rear edge, which is slightly wider than the front
//
right_side_edge_offset=3.7;     // Edge connector slot, from right side
left_side_edge_offset=22.2;     // Edge connector slot, from left side

spacing=0.33;                   // Bit of slack to ensure things fit together

base_width=side_wall_thickness + spacing + pcb_width + spacing + side_wall_thickness;
base_length=closest_wall_thickness + spacing + pcb_length + spacing + far_wall_thickness;

pcb_aperture_width = spacing + pcb_width + spacing;
pcb_aperture_length = spacing + pcb_length + spacing;

locating_post_height=5;
locating_post_hole_diameter=4.3;
locating_post_diameter=locating_post_hole_diameter-spacing;
locating_post1_xpos_centre=19.11;
locating_post1_ypos_centre=11.01;
locating_post2_xpos_centre=69.2;
locating_post2_ypos_centre=84.6;

$fn=64;

// Build the shell
//
difference()
{
  // Basic cube
  //
  cube([base_width,base_length,walls_height], false);
  
  // Cut out the middle, PCB drops into here
  //
  translate([side_wall_thickness, closest_wall_thickness, base_thickness])
  {
    cube([pcb_aperture_width,pcb_aperture_length,walls_height], false);
  }
  
  // Cut out front wall for the edge connector
  //
  edge_xpos = side_wall_thickness + spacing + left_side_edge_offset;
  edge_width=pcb_width - left_side_edge_offset - right_side_edge_offset;
  translate([edge_xpos, 0, base_thickness])
  {
    cube([edge_width,closest_wall_thickness,walls_height], false);
  }

  // Cut out rear wall for the passthrough connector
  //
  far_wall_ypos=closest_wall_thickness + spacing + pcb_length + spacing;
  translate([edge_xpos, far_wall_ypos, base_thickness])
  {
    cube([edge_width,far_wall_thickness,walls_height], false);
  }
}

// Centring the posts centres them vertically as well, so use this instead
//
locating_post_offset=locating_post_diameter/2;

translate([side_wall_thickness+spacing+locating_post1_xpos_centre-locating_post_offset,
           closest_wall_thickness+spacing+locating_post1_ypos_centre-locating_post_offset,
           0])
{
  cylinder(locating_post_height, locating_post_diameter/2, locating_post_diameter/2, false);
}

translate([side_wall_thickness+spacing+locating_post2_xpos_centre-locating_post_offset,
           closest_wall_thickness+spacing+locating_post2_ypos_centre-locating_post_offset,
           0])
{
  cylinder(locating_post_height, locating_post_diameter/2, locating_post_diameter/2, false);
}