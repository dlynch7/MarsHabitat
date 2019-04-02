# MarsHabitat
Mostly robotics code for NU's research on 3D printing structures with Marscrete.

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

There are many other things worth working on, within motion control and planning:
* Inverse velocity kinematics
* Modeling the dynamics of the arm with the extruder attached
* Formulating the path-planning and inverse kinematics as a constrained nonlinear optimization problem, and solving it with MATLAB's `fmincon`. This includes constraints on
    * Extruder position, as determined by the workspace (windows, workpiece, etc)
    * Position of each joint in the task space (again, windows, ceiling, workpiece, etc)
    * Limits on joint position, velocity, and acceleration


### Extruder: modeling and thermal control
#### Current status
I know few details about the extruder's thermal control system. Here's what I know:
* The purpose of the thermal control system is to maintain temperature of the Marscrete mixture at 140 C (at which the sulfur in the mixture melts). I believe sulfur ignites at 200 C, and an ignition destroyed the previous extruder (a prototype).
* The current extruder uses wire heating elements that work via Joule heating. These heating elements are driven by some kind of COTS control box. Previously, there was only one temperature sensor (a thermocouple) mounted on the extruder and connected to the controller, but I believe there is now at least one additional thermocouple mounted elsewhere on the extruder.
* Poor thermal control is partly responsible for inconsistent extrustion and inconsistent Marscrete quality.
* I think there is some kind of load cell under the mini-hopper that is mounted to the extruder assembly, but I don't know anything else about it. The reading from this load cell is compared to a threshold; when it falls below that threshold, the mini-hopper must be refilled.

I don't think there is a thermal model yet, but it is necessary.

#### Path forward
* Obtain thermal and fluid properties of the Marscrete mixture. I believe the mixture consists mostly of sulfur and basalt, but I do not know their amounts or what else is in the mixture.
    * Viscosity, specific heat, density, etc.
* Develop an open-loop thermal model where the control input is heat flux as a function of time (and possibly of position).
* Develop an open-loop thermal model where the control input is the material flow rate.
    * Analyze controllability and observability of the open-loop thermal models
    * How does observability depend on the number of temperature sensors and their placement?
* Determine if Joule heating is the right way to add heat to the mixture.
For example, would a fluid-based heat exchanger be better?
* (Assuming some degree of controllability) design a control law to keep the mixture molten without reaching the ignition temperature.
* (Assuming some degree of observability) design an observer to estimate the temperature profile of the material throughout the extruder.
* Implement the controller/observer on an embedded microprocessor. This task should be coordinated with systems integration to ensure the embedded implementation can interface with the robot's master controller.
* It may be worth sensing the temperature of the extruder motor, since it can stall and overheat when a clog occurs.

One last task that would be very cool and useful, although beyond the current scope, is detection of clogs and automatic declogging.

### System integration and high-level control
#### Current status
The robot arm subsystem and extruder subsystem have not been integrated yet.

#### Path forward
I believe ABB's RAPID command language will help here.
RAPID is a modular command language, and each module (called a procedure) can call other modules.
Also, RAPID provides an interface to standard networking sockets, which may be a viable way to interface the main computer with the embedded microprocessor on which the extruder controller is implemented.

Therefore, it seems possible to write a RAPID procedure that implements a finite state machine which acts as the master controller, stepping through arm motion commands in one subordinate procedure while listening/polling the extruder controller over a network socket. Depending on the extruder status (nominal, empty, too hot, too cold, etc), this finite state machine can pause/resume arm motion along the print path as well as instruct the arm to move to a non-print location (for refilling, for example).
