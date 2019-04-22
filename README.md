# Stewart Platform Simulation
Simulation environment of MATLAB and Processing is used to model the Stewart Platform problem. The MATLAB model is for graphing leg actuation. This code can ensures that the real simulator legs don't exceed maximum excursion within the operational envelop. The Processing model was made to study motion cueing methods of the simulator problem.


## How to run
Open "StewLegLog.m" and press F5. A GUI will pop up and you can edit the attutide or translation of the simulator and view the corresponding leg actuation.

Note: Follow the color scheme for the rainbow "roygbv", leg No. 1 is red dashed, leg No. 2 is yellow, leg 3 No. 3 is green and etc.

## Illustrations
<img src="https://github.com/Tac321/Stewart-Platform/blob/master/Images/Stewart_MATLAB.png" width="700" />

Below, the pose information was extracted from an Arduino IMU and pushed through the Stewart simulation to extract approximate leg actuations, neglecting simulator translation actuation.



<img src="https://github.com/Tac321/Mean-Shift-Tracking-MST-/blob/master/Picture3.jpg" width="700" />



<img src="https://github.com/Tac321/Stewart-Platform/blob/master/Images/meanSmoothedPoseExtrimumExcursionTestTrajectory_10_10_2018.jpg" width="700" />


Below are the corresponding leg actuations needed to mimic the poses of the above plot.

<img src="https://github.com/Tac321/Stewart-Platform/blob/master/Images/LegExcursionsExtrimumExcursionTestTrajectory_10_10_2018.jpg" width="700" />



A Processing code version of the Stewart platform is attached. This algorithm acts to track a rolling ball to a user defined position on the platform plate. Note: The ball model is not an exact nonlinear model of a sphere, this higher fidelity is to be added later. 

<img src="https://github.com/Tac321/Stewart-Platform/blob/master/Images/StewartMotionCue.gif" width="700" />


## How to run

# Before running code
1) Install libraries ControlP5, Peasy, and oscP5. Youtube videos explain how to do this.

## Run code
1) Open "Stewart_PlatformProcessing.pde"
2) Run the program
3) Press the following in sequence "g', 'h', 'j' 
4) Use the bottom two right side sliders to select desired marble positions on the platform, so the controller can maneuver the ball on the platform.

