# Stewart Platform Simulation
Simulation environment of MATLAB is used to model Stewart Platform, for graphing leg actuation. This code can ensure that the real simulator legs don't exceed maximum excursion within the operational envelop.


## How to run
Open "StewLegLog.m" and press F5. A GUI will pop up and you can edit the attutide or translation of the simulation and view the leg actuation.

Note: Follow the color scheme for the rainbow "roygbv", leg No. 1 is red dashed, leg No. 2 is yellow, leg 3 No. 3 is green and etc.

## Illustrations
<img src="https://github.com/Tac321/Stewart-Platform/blob/master/Images/Stewart_MATLAB.png" width="700" />

Below, the pose information was extracted from the Arduino IMU and pushed through the Stewart simulation to extract approximate leg actuations, neglecting simulator translation.


<img src="https://github.com/Tac321/Stewart-Platform/blob/master/Images/meanSmoothedPoseExtrimumExcursionTestTrajectory_10_10_2018.jpg" width="700" />


Below are the corresponding leg actuations needed to mimic the poses of the above plot.

<img src="https://github.com/Tac321/Stewart-Platform/blob/master/Images/LegExcursionsExtrimumExcursionTestTrajectory_10_10_2018.jpg" width="700" />


A Processing code version of the Stewart platform is attached. This algorithm acts to track a rolling ball to a user defined position of the platform plate. Note: The ball model is not an exact nonlinear model of a sphere, this higher fidelity is to be added later. 

<img src="https://github.com/Tac321/Stewart-Platform/blob/master/Images/StewartMotionCue.gif" width="700" />


## How to run
Open "SURVICE_Stewart_Platform.pde"  and run the file such that...

## MPCC
Simulation environment of the Model Predictive Contouring Controller (MPCC) for Autonomous Racing developed by the Automatic Control Lab (IfA) at ETH Zurich.

# Note
Code developed originally by Dr. Alex Liniger et al. My repository only uses an alternative to Quadprog(), (Quadratic Problem optimization solver) . My QP solver was coded from scratch.


## How to run

# Before running code
1) Install libraries ControlP5, Peasy, and oscP5. Youtube videos explain how to do this.

## Run code
1) Open "Stewart_PlatformProcessing.pde"
2) Run the program
3) Press the following in sequence "g', 'h', 'j' 
4) Use the bottom two right side sliders to select desired marble positions of the platform, so the controller can maneuver the ball on the platform.

