width=19.0;
length=22.5;
height=2.5; 

difference()
{
  cube([width, length, height]);
  
  translate([3, 3, 0])
  {
    cube([width-3-3, length-3-3, height]);
  }
}

  