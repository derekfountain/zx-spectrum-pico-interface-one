The board for prototype 2 fits across the back of the Spectrum.
It sits between the power plug and the MIC plug. The power side
is spot on; the MIC side is slightly too wide, it protrudes
into the socket area just by 1mm or so.

So the prototype board is very slightly wider than the entire
outer width the case needs to be.

The board is 118.95mm wide according to KiCAD. Calipers agree,
saying 118.92mm.

Assume a 2mm case thickness and 0.5mm spacing, width of board
needs to be:

 118.95 - 1.00 = 117.95
 117.95 - 2.00 - 0.50 - 2.00 - 0.50 = 112.95mm

The edge connector in the prototype is perfectly positioned at
10.37mm from the right side of the edge cut. Bring in the right
side edge cut by 2.5mm. Rear edge connector needs to be the same.

The left side edge cut needs to come in a bit further. The board
is 6mm narrower than the prototype, and it's lost 2.5mm on the
right side, so it needs to lose 3.5mm on the left side.

The edge connector height is correct as well, that's 2.43mm from
the lower edge cut.


Front to back of the board, and hence the case, will depend on
how tight I can get the layout. I'll come back to that. I need
to lay the rear edge connector before doing anything else though.


Height, assuming Picos in header sockets, would be about 20mm
from tip of pin on underside to top surface of Pico. The Spectrum
is just over 30mm, so I'll make the case flush with the top of
the Spectrum.


I'd like a nice aluminium panel for the top surface, with drilled
LED holes and screen print. These are cheap on JLC, just remember
to turn off the testing. Round the corners, needs 2 or 4 screw
holes.


From TomD:

"My side walls are 2mm thick and I find this strong enough for most
use cases. The case does have a little flex but that is preferred to
something totally rigid. Also need to be careful on the part closest
to the Spectrum as too thick will mean the connector cannot go in far
enough. I made the top and bottom 1mm thick to give as much room as
possible. Overall the case isn't IF2 sturdy, which is probably 3mm
thickness, but easily good enough."

Good point about the edge connector fouling.