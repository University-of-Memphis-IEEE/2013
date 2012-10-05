// readme.txt
Author: Dustin Maki

Purpose of the code:
Make sure all motors are connected properly, wheels turn in the expected directions, and the robot moves as intended.
Each wheel will, in turn, move in the forward direction as indicated in the sketch comments. 
Next, each wheel will, in turn move in the reverse direction.  The order of progression is Front. Rear, Left, Right.
Then, the entire robot, while remaining facing in one direction, will move in an octagonal pattern.
This is highly unlikely to work as intended on the first try.
Identify which wheel actually moves when a given wheel is commanded.
Based on those results, any motors that respond to the wrong commands, unplug them and plug them in to the appropriate motor port.
Run the test code again.  Repeat until the wheels move in the proper progression Front. Rear, Left, Right.
If that fails or is difficult due to the constrained wiring space, at least ensure that the front and rear wheels are connected to one motor controller while the left and right wheels are connected to the other motor controller.
Once that is confirmed,
If front or rear move when left or right are commanded, then the addresses in the code are reversed.  In configure.h lines 5 and 6, swap the address assignments.
If front moves when rear should, then the speed registers are reversed.  In configure.h lines 10 and 11, swap the register numbers 0 and 1.
If left moves when right should, then the speed registers are reversed.  In configure.h lines 8 and 9, swap the register numbers 0 and 1.
Now, the wheels should all be turning in the correct sequence.  It is time to get them turning in the correct direction.
If the front and rear wheels turn in the same direction, but the direction is wrong, go through all moveXX() functions at the end of configure.h and reverse the sign of both the first and second parameters in all calls to drive4wheelSpeeds(-speed, -speed, speed, speed).  Be sure to change the comment in omnitest.ino to reflect the change of axis polarity.
If only one wheel of the pair moves in the wrong direction, then we may need to change to different methods.
The same procedure applies to left and right wheels, however, it is the third and fourth parameters that would change.
Once this is done, following the single wheel move sequence, the bot should move in an octagon pattern and end at roughly the same place it started.

The code referenced in this readme resides at
 https://umdrive.memphis.edu/g-IEEE-student-branch/robot
 https://github.com/University-of-Memphis-IEEE/2013/omnitest
 