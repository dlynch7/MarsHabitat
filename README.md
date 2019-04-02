# MarsHabitat
Mostly robotics code for NU's entry in the 2019 NASA Mars Habitat 3D printing challenge

------------------------

## Upcoming tasks

### Robot arm kinematics and path-planning
#### Current status
Currently, we are using ABB's RobotStudio (a proprietary closed-source Windows-only program) to calculate joint trajectories given a commanded tool path. This method is restrictive:
* RobotStudio does not appear to allow arbitrary positions of the outermost joint. Instead, this joint angle can either be fixed in the task frame, or it can follow the tool-frame path. This might be okay for applications featuring a small tool, but it does not work for our application, because our extruder is over 1 m long.
* This restriction on the outermost joint angle limits the size and placement of the structures we can print with the arm.

* I have developed [a simple MATLAB script](ABB_IRB_6700_155_2848/kinematics/cylWallTask.m) that solves the inverse kinematics problem for a given end-effector pose, using a modified version of the Newton-Raphson root-finding algorithm presented in [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics) by Lynch and Park. [Here](https://drive.google.com/file/d/1toldz0SdsCVAZ9mCjw-wtX-4AoiSo1Hw/view?usp=sharing) is a short simulation demonstrating this script.

#### Path forward
While we are restricted to using RobotStudio to generate RAPID files which the arm takes as instructions, there may be a workaround that lets us use the full range of motion of the outermost joint.

* If RobotStudio supports the [M19](http://www.helmancnc.com/m19-spindle-orientation-m119-sub-spindle-orientation/) G Code command (spindle orientation), it should be possible to expand the MATLAB script to generate G Code and assign the outermost joint angle to that M19 code.

* If RobotStudio does not support the M19 command, we will have to contact their tech support team for help.


### Extruder: modeling and thermal control
#### Current status

#### Path forward


### System integration and high-level control
#### Current status
#### Path forward
