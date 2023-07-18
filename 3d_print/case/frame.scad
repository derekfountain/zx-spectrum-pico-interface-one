case_aperture_width=30.0;
case_aperture_length=31.0;

side_border_width=1.0;
top_border_width=3.0;
bottom_border_width=3.0;

frame_width=side_border_width + case_aperture_width + side_border_width;
frame_length=top_border_width + case_aperture_length + bottom_border_width;

screen_width=27.0;
screen_length=15.0;

screen_hole_xpos=(frame_width-screen_width)/2;
screen_hole_ypos=bottom_border_width + 10.0;

frame_height=0.5;

difference()
{
  cube([frame_width, frame_length, frame_height]);

  // Knock out the screen hole
  //
  translate([screen_hole_xpos, screen_hole_ypos, 0])
  {
    cube([screen_width, screen_length, frame_height]);
  }
}

translate([0, 0, frame_height])
{
  frame_raise=1.5;
  difference()
  {
    cube([frame_width, frame_length, frame_raise]);
    
    translate([side_border_width, bottom_border_width, 0])
    {
      #cube([frame_width-(side_border_width*2),
            frame_length-bottom_border_width-top_border_width,
            frame_raise]);
    }
  }
}