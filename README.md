# Robot Guiding And Navigation using robot simulator APOLO and MATLAB

Main implementations:

-Location:Extended Kalman Filter

-Path planning:RRT-connect (Rapidly-exploring random tree)

-Control: Reactive control using ultrasonic sensors.



This work has three main parts:

-Robot location. For this part, an Extended Kalman Filter has been implemented, in order to be able to locate the robot. Noise in position is applied by the APOLO program so the EKF take it in count.
The robot also has a laser sensor that triangulates the position thanks to the laser beacons.

-Path planning. For this part, both RRT and RRT-Connect algorithms have been implemented. They plot the tree construction 'in real time' and its final solution path.

-Movement control. The movement is simulated using  the solution path and has Reactive Control to unexpected objects on its trajectory


The code is developed using the program APOLO, and MATLAB.
APOLO is a robot simulator that allows the simulation of this project.


Project authors:
Adrián Caro 
Hugo Rajado 
Pablo Dopazo
