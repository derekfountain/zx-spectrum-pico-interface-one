board_width=99.9;               // Width of board, from KiCAD
board_length=100.0;             // Length of board, from KiCAD
side_inner_spacing=0.15;        // This much either side, I want a friction fit
front_inner_spacing=0;
rear_inner_spacing=0.15;

// Copy these from the lower case SCAD, keep them in sync
//
lower_case_side_wall_thickness=2.0;
lower_case_closest_wall_thickness=0.0;
lower_case_spacing=0.33;
lower_case_far_wall_thickness=1.0;
lower_case_walls_height=25.5;  // temp, this is the height of the walls on the last one i printed
lower_case_sd_card_width=13.0;
lower_case_sd_card_offset=8.0;

// This is the width/length of the inside of the upper case
//
inner_width = side_inner_spacing +
              lower_case_side_wall_thickness + lower_case_spacing +
              board_width +
              lower_case_spacing + lower_case_side_wall_thickness + 
              side_inner_spacing;
inner_length= front_inner_spacing +
              lower_case_closest_wall_thickness + 0 +
              board_length +
              lower_case_spacing + lower_case_far_wall_thickness +
              rear_inner_spacing;

side_wall_thickness=2.0;        // Left and right side walls
far_wall_thickness=2.0;         // Wall at rear of device, might obstruct plugging in
closest_wall_thickness=0.0;     // Wall closest to ZX

lid_thickness=1.5;              // This needs to be strong

// This is the outermost dimension of the "lid". The case height here is 
// is the height of the walls which should match the entire height of the
// lower case. I subtract a small amount to ensure the top is slightly
// shorter than the base.
//
case_width =  side_wall_thickness + inner_width + side_wall_thickness;
case_length = closest_wall_thickness + inner_length + far_wall_thickness;
case_height = lower_case_walls_height + lid_thickness - 0.75;


difference()
{
  // Main box, wraps the whole device
  //
  cube([case_width, case_length, case_height]);
  
  // Knock out the inner area
  //
  translate([side_wall_thickness, closest_wall_thickness, 0])
  {
    cube([inner_width, inner_length, case_height-lid_thickness]);
  }
  
  // Knock out the lower far wall to allow access to the rear edge connector
  //
  translate([side_wall_thickness, case_length-far_wall_thickness, 0])
  {
    cube([inner_width, far_wall_thickness, case_height-14.0]);
  }

  // Knock out the lower side wall where the SD card pokes through
  //
  translate([0, lower_case_sd_card_offset, 0])
  {
    cube([side_wall_thickness, lower_case_sd_card_width, 11]);
  }
  
  
  // Big hole for the face plate
  //
// This version cuts a hole in the right place of the right size
//
//  plate_xpos=side_wall_thickness +
//             side_inner_spacing +
//             lower_case_side_wall_thickness +
//             lower_case_spacing +
//             5.0;                               // Calliper measurement
//  plate_ypos=closest_wall_thickness +
//             front_inner_spacing +
//             lower_case_closest_wall_thickness +
//             lower_case_spacing +
//             7.0;                               // Calliper measurement
//  translate([plate_xpos, plate_ypos, case_height-lid_thickness])
//  {
//    // Plate size is from KiCAD
//    //
//    cube([90.1, 24.3, lid_thickness]);
//  }

  // Underlap is the horizontal distance the lid fits under the face plate
  // all round. 
  //
  underlap=2.0;
  plate_xpos=side_wall_thickness +
             side_inner_spacing +
             lower_case_side_wall_thickness +
             lower_case_spacing +
             underlap + 
             5.0;                               // Calliper measurement
  plate_ypos=closest_wall_thickness +
             front_inner_spacing +
             lower_case_closest_wall_thickness +
             lower_case_spacing +
             underlap +
             7.0;                               // Calliper measurement
  translate([plate_xpos, plate_ypos, case_height-lid_thickness])
  {
    // Plate size is from KiCAD
    //
    cube([90.1-(underlap*2), 24.3-(underlap*2), lid_thickness]);
  }

  
  // Hole for the OLED
  //
  oled_xpos=side_wall_thickness +
            side_inner_spacing +
            lower_case_side_wall_thickness +
            lower_case_spacing +
            35.5;                               // Calliper measurement
  oled_ypos=closest_wall_thickness +
            front_inner_spacing +
            lower_case_closest_wall_thickness +
            lower_case_spacing +
            55.5;                               // Calliper measurement
  translate([oled_xpos, oled_ypos, case_height-lid_thickness])
  {
    // OLED size is from callipers
    //
    cube([28.5, 28.5, lid_thickness]);
  }
  
  // Hole for cancel button
  //
  cancel_xpos=side_wall_thickness +
              side_inner_spacing +
              lower_case_side_wall_thickness +
              lower_case_spacing +
              65.9;                               // Calliper measurement
  cancel_ypos=closest_wall_thickness +
              front_inner_spacing +
              lower_case_closest_wall_thickness +
              lower_case_spacing +
              52.0;                               // Calliper measurement
  translate([cancel_xpos, cancel_ypos, case_height-lid_thickness])
  {
    cube([6.6, 6.5, lid_thickness]);
  }
  
  // Hole for action button
  //
  action_xpos=side_wall_thickness +
              side_inner_spacing +
              lower_case_side_wall_thickness +
              lower_case_spacing +
              83.5;                               // Calliper measurement
  action_ypos=closest_wall_thickness +
              front_inner_spacing +
              lower_case_closest_wall_thickness +
              lower_case_spacing +
              52.0;                               // Calliper measurement
  translate([action_xpos, action_ypos, case_height-lid_thickness])
  {
    cube([6.5, 6.5, lid_thickness]);
  }

  // Rotary knob
  //
  knob_diameter=15.0;
  knob_spacing=1.0;      // All round
  knob_hole_diameter=knob_diameter + knob_spacing;
  knob_hole_radius=knob_hole_diameter/2;
  knob_xpos=side_wall_thickness +
             side_inner_spacing +
             lower_case_side_wall_thickness +
             lower_case_spacing +
             knob_spacing +
             knob_hole_radius +
             74.85;                               // Calliper measurement
  knob_ypos=closest_wall_thickness +
            front_inner_spacing +
            lower_case_closest_wall_thickness +
            lower_case_spacing +
            knob_spacing +
            knob_hole_radius +
            72.5;                               // Calliper measurement
  translate([knob_xpos, knob_ypos, case_height-lid_thickness])
  {
    cylinder(lid_thickness, knob_hole_radius, knob_hole_radius, false);
  }
  
}

// Add an inner track to help locate the top
//
// This needs the lower case size walls to extend up a couple of mm
// so the walls fit in the slots. Changed my mind, I want the lid to
// sit flush on the top of the walls.
//
//tracks_width     = 2;
//tracks_length    = board_length - rear_inner_spacing;
//left_track_xpos  = side_wall_thickness +
//                   side_inner_spacing +
//                   lower_case_side_wall_thickness;
//right_track_xpos = case_width -
//                   side_wall_thickness -
//                   lower_case_side_wall_thickness -
//                   side_inner_spacing - 
//                   tracks_width;
//tracks_ypos      = closest_wall_thickness +
//                   front_inner_spacing +
//                   lower_case_closest_wall_thickness + 0;
//
//translate([left_track_xpos, tracks_ypos, 0])
//{
//  cube([tracks_width, tracks_length, case_height]);
//}
//
//difference()
//{
//  translate([right_track_xpos, tracks_ypos, 0])
//  {
//    cube([tracks_width, tracks_length, case_height]);
//  }
//
//  // Solder point for the 3 pin header is in the way!
//  //
//  translate([right_track_xpos, 58, -lid_thickness])
//  {
//    cube([tracks_width, 11, case_height]);
//  }
//}
//
//translate([left_track_xpos, tracks_length-tracks_width, 0])
//{
//  cube([right_track_xpos-left_track_xpos+tracks_width, tracks_width, case_height]);
//}