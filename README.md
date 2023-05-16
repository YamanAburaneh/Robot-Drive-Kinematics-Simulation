# Python Simulation for Rear Wheel Drive Ackermann Steering Robot and Differential Drive Robot


## Rear Wheel Drive Ackermann Steering Robot Simulation
- This folder contains two files:
  1) The first one is the main code for the first assignment task (Ackermann_Steering_Robot_Main.py)
  2) The second one is fully controlable Ackermann Steering simulation (Ackermann_Steering_Robot_Control.py)
- The control buttons are printed on the screen.
- Uses front wheel Ackermann steering kinematics to control the direction of the robot.
- Draws the turing radius while steering and the axle lines that intersect the center of that radius (IC).

## Differential Drive Robot Simulation
- This file will display the required output when it is launched (Vl = V + V*15% , Vr = V - V*15%).
- It is possible to control the robot using: 
  (E: Accelerate right side, D: Deccelerate right side, Q: Accelerate left side, A: Deccelerate left side,
   W: Drive forward at an average speed of the two wheels, S: Stop).
- The robot will rotate around its center point when the speeds of both sides are the same but in opposite directions.


### The chosen configuration for both robots:
- Distance between axles, L = 1.0m
- Axle width, 		  W = 0.5m
- Drive Velocity,	  V = 3.0m/s

Both simulations have been developed using Pygame and Math modules in python.

To install Pygame --> command in terminal --> pip install pygame





