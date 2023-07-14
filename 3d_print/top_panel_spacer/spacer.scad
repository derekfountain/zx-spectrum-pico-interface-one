width=70;
depth=20;
height=2.0;

cutout_width=58;
cutout_depth=8;

difference()
{
  cube( [width, depth, height] ); 
  
  // Knock out middle strip for the LEDs
  //
  translate( [(width-cutout_width)/2, (depth-cutout_depth)/2, 0] )
  {
    cube( [cutout_width, cutout_depth, height] );
  }

  // Knock out gap for the header connector. Caliper measurements.
  //
  translate( [13, ((depth-cutout_depth)/2)+cutout_depth, 0] )
  {
    cube( [5, 8, height] );
  }
  
}
