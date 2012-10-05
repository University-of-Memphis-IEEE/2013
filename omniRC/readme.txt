Author: Dustin Maki

omniRC allows connection of a hobby Radio Control receiver to allow teleoperation of the 4 wheel omni robot.
In this case, the RC system consists of a 6 channel transmitter/receiver pair from Vex robotics version 0.5.
The receiver has been modified to accept the voltage available on the robot.  
The receiver output is a PPM modulated pulse train.  It is connected to the input compare pin on the AVR microcontroller which runs this code.