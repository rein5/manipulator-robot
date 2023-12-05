# manipulator-robot
Matlab implementation of a 2-link differential-drive wheeled manipulator robot.

Final project for AUE-8220.

## Matlab Scripts

implementations for individual problems are as follows

- *p2_1.m*: open loop control with pseudoinverse solution
- *p2_2_1.m*: redundancy resolution with added constraint theta1dot + theta2dot = 0
- *p2_2_2.m*: redundancy resolution with added constraint theta1dot = 0
- *p2_3.m*: potential minimization 
- *p3.m*: closed-loop task-space control

Each of the above will compute the corresponding solution and then display the result with a simple animation. Additional plots are also produced (desired end-effector trajectory, etc.)

## Matlab App 

The matlab app *robot_app.mlapp* also implements all of the above control methods, but with the benefit of a GUI and a better animation. The application also allows for the setting of some of the robot parameters, as well as for the switching between the end-effector trajectories.


### Dependencies

The app *robot_app.mlapp* requires the Image Processing Toolbox to be installed. 
