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

// The device PCB lies flush with the bottom plastic of the Spectrum's case. 
// Given there are the soldered tips of the through hole devices poking through
// the PCB, and that the PCB must be raised up about 4mm at the rear to give
// clearance for another interface to be plugged in rear, the PCB needs to stand
// clear of the base of its case by about 4mm.
// This means the rear of the Spectrum will need to be raised up and supported
// from underneath, like the IF1 but on a smaller scale. I'll create a new
// support for that.
//
standoff_height=5;

locating_post_height=standoff_height + 8;
locating_post_hole_diameter=4.3;
locating_post1_xpos_centre=19.11;
locating_post1_ypos_centre=11.01;
locating_post2_xpos_centre=69.2;
locating_post2_ypos_centre=84.6;

sd_card_offset=9.0;             // From front edge of board to SD card reader
sd_card_width=13.0;             // Width of the SD card reader I'm using, inc solder tabs

switches_offset=3.75;           // From left side to left edge of first switch

// This needs to be about 16mm, but that leaves a thin little post between the
// switches aperture and the rear edge connector aperture. That will be hard to
// print with any strength. So I made the switches aperture a bit wider to open
// it out.
//
switches_width=20.0;            // Width of aperture to allow both switch buttons through

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
  edge_wall=3.0;
  translate([edge_xpos, 0, base_thickness+edge_wall])
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
  
  // Cut out side wall where the SD card reader pokes through
  //
  sd_cutout_wall_height=4;
  translate([0, sd_card_offset, base_thickness+sd_cutout_wall_height])
  {
    cube([side_wall_thickness,sd_card_width,walls_height], false);
  }
  
  // Cut out rear wall where the reset switches poke through. Leave a 
  // small amount of the wall to help locating rear interfaces
  //
  switches_aperture_edge=side_wall_thickness+spacing+switches_offset;
  switches_wall_height=standoff_height+2;
  translate([switches_aperture_edge, far_wall_ypos, base_thickness+switches_wall_height])
  {
    #cube([switches_width,far_wall_thickness,walls_height], false);
  }
  
}

// Make posts slightly smaller than the hole so device fits over it
//
locating_post_diameter=locating_post_hole_diameter-spacing;

translate([side_wall_thickness+spacing+locating_post1_xpos_centre,
           closest_wall_thickness+spacing+locating_post1_ypos_centre,
           0])
{
  cylinder(locating_post_height, locating_post_diameter/2, locating_post_diameter/2, false);
}

translate([side_wall_thickness+spacing+locating_post2_xpos_centre,
           closest_wall_thickness+spacing+locating_post2_ypos_centre,
           0])
{
  cylinder(locating_post_height, locating_post_diameter/2, locating_post_diameter/2, false);
}

// Add some blocks to raise the PCB up a bit. Location of these isn't really
// important, one in each corner will do. But getting one under the rotary
// will a bit of strength there. 
//
block_width=side_wall_thickness + spacing + 5;
block_length=closest_wall_thickness + spacing + 5;
block_height=standoff_height;

// Front left
//
block1_xpos=10;
block1_ypos=25;
translate([block1_xpos, block1_ypos, base_thickness])
{
  cube([block_width, block_length, block_height], false);
}

// Rear left
//
block2_xpos=13;
block2_ypos=70;
translate([block2_xpos, block2_ypos, base_thickness])
{
  cube([block_width, block_length, block_height], false);
}

// Front right
//
block3_xpos=78;
block3_ypos=25;
translate([block3_xpos, block3_ypos, base_thickness])
{
  cube([block_width, block_length, block_height], false);
}

// Rear right
//
block4_xpos=83;
block4_ypos=77;
translate([block4_xpos, block4_ypos, base_thickness])
{
  cube([block_width, block_length, block_height], false);
}

