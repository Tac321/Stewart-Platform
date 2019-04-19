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

