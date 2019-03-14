%% cylWallTask.m
%
% Description:
%   Simulate the kinematics (forward, inverse) required to print a
%   cylindrical wall in the workspace in Tech AG53 with the ABB IRB 6700
%   arm.
%
% Notes:
%   1 - all linear dimensions are in meters except where otherwise noted.
% Dependencies:
%   1 - kinematics library from Modern Robotics by Lynch & Park:
%       https://github.com/NxRLab/ModernRobotics/tree/master/packages/MATLAB
%   2 - stlread.m (imports STL file as a struct with fields "faces" and
%   "vertices" - super handy
%
% Author:
%   Dan Lynch
%
% Date:
%   March 9, 2019

%% main function
function cylWallTask
    clear all;
    close all;
    clc;
    
    % add Modern Robotics library to path:
    addpath('/home/daniel/ModernRobotics/packages/MATLAB/mr');
    
    % add stlread.m to path:
    addpath([pwd,'/../../STL_and_related/STLRead']);
    
    % add fold with STLs and G-Code to path:
    % add stlread.m to path:
    addpath([pwd,'/../../STL_and_related/stls_and_gcode']);
    
    % declare presistent structs:
    persistent perim slab robot workpiece
    
    % initialize the workspace and the robot:
    [perim,slab,robot,workpiece] = init_workspace(perim,slab,robot,workpiece);
    robot.nJoints = 6;
    robot = init_robot(robot);
    workpiece = gen_pts(workpiece);
    
    robot.thetalist = vertcat(deg2rad([30;60;-30;0;60;0]));
    
    px = 4.5;
    py = 3.0;
    pz = 0.0;
    
    R_ext = [0,0,1;
              1,0,0
              0,1,0];
    yaw = 0; % radians
    R_yaw = [cos(yaw), -sin(yaw), 0;
             sin(yaw),  cos(yaw), 0;
             0,         0,        1];
    
    taskspace_pose = horzcat(vertcat(R_yaw*R_ext, [0,0,0]),[px;py;pz;1]);
%     taskspace_pose = [0,0,1,px;
%                       1,0,0,py;
%                       0,1,0,pz;
%                       0,0,0,1];
    [thetalist,robot_pose,exitflag] = robot_NRIK(taskspace_pose,robot);
    [at_lim,thetalist_sat] = at_joint_lim(thetalist,robot)
    robot.thetalist = thetalist;
    
    % visualize everything:
    visualize_workspace(perim,slab,robot,workpiece);
end

%% Check if any joint angles are at their limits
function [at_lim,thetalist_sat] = at_joint_lim(thetalist,robot)
    for i = 1:robot.nJoints
       if thetalist(i) < robot.angles.joints_min(i)
           at_lim(i) = -1; % -1 --> joint i is at lower limit
           thetalist_sat(i) = robot.angles.joints_min(i); % saturate
           fprintf('theta(%d) = %f, limit is %f.\n',i,...
               rad2deg(thetalist(i)),rad2deg(robot.angles.joints_min(i)));
       elseif thetalist(i) > robot.angles.joints_max(i)
           at_lim(i) = 1; % 1 --> joint i is at upper limit
           thetalist(i) = robot.angles.joints_max(i); % saturate
           fprintf('theta(%d) = %f, limit is %f.\n',i,...
               rad2deg(thetalist(i)),rad2deg(robot.angles.joints_max(i)));
       else
           at_lim(i) = 0; % 0 --> joint is within limits
           thetalist_sat(i) = thetalist(i); % joint i value is unchanged
       end
    end
end

%% Compute inverse kinematics (using optimization) for a pose in task space:
function [thetalist, robot_pose, exitflag] = robot_optIK(taskspace_pose,robot,perim)
% Description:
%   Given a desired end-effector location in task space, robot_optIK 
%   find the 7 joint positions and end-effector orientation by solving a 
%   nonlinear constrained optimization problem.
%
% Optimization problem:
%   Input vector:
%       "x" = [q1; q2; q3; q4; q5; q6; ext; theta];
%       "q1", ..., "q6":    robot arm joint angles [radians] (bounded above
%       and below to reflect joint limits)
%
%       "ext":        extruder position (discrete variable - extruder is
%       either extended (ext = 0) or retracted (ext =
%       -robot.extruder.length)
%
%       "theta": rotation angle [radians] of extruder about the task-space
%       vertical axis
%   Constraints:
%       "px", "py", "pz": commanded end-effector position in the task-space
%       frame
%
%       perimeter: no part of the robot, including the extruder, may be
%       outside the x-y-z perimeter of the workspace. Nonlinear inequality
%       constraint.
%
%   Objective function:
%       Minimize the weighted inner-product of the twist from the current
%       position determined by thetalist_cand to the desired end-effector
%       pose "taskspace_pose". The weight matrix Q is 6x6 diagonal matrix
%       that weights wx, wy, wz, vx, vy, vz (in that order).
%       

end

%% Weighted pseudoinverse for "fat" matrices (more columns than rows)
function [J_pinv] = weighted_pinv(J,W)
    if (size(J,2) == size(W,1)) && (~(ismatrix(W) && diff(size(W))))
        J_pinv = (W\(J'))/((J/W)*(J'));
    elseif (size(J,2) ~= size(W,1))
        error('dimension 2 of J and dimension 1 of W do not match.');
    elseif (~(ismatrix(W) && diff(size(W))))
        error('W is not a square matrix.');
    else
        error('dimensions of J and W do not match.');
    end
end

%% Modified Newton-Raphson inverse kinematics for a pose in task space:
function [thetalist, success] ...
         = IKinSpaceFreewz(Slist, M, T, thetalist0, eomg, ev, W)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Slist: The joint screw axes in the space frame when the manipulator
%              is at the home position, in the format of a matrix with the
%              screw axes as the columns,
%       M: The home configuration of the end-effector,
%       T: The desired end-effector configuration Tsd,
%       thetalist0: An initial guess of joint angles that are close to 
%                   satisfying Tsd,
%       eomg: A small positive tolerance on the end-effector orientation 
%             error. The returned joint angles must give an end-effector 
%             orientation error less than eomg,
%       ev: A small positive tolerance on the end-effector linear position 
%           error. The returned joint angles must give an end-effector 
%           position error less than ev.
%       W: a 6x6 (nominally diagonal) matrix that weights the
%           descent direction used in the Newton-Raphson algorithm.
% Returns thetalist: Joint angles that achieve T within the specified 
%                    tolerances,
%         success: A logical value where TRUE means that the function found
%                  a solution and FALSE means that it ran through the set 
%                  number of maximum iterations without finding a solution
%                  within the tolerances eomg and ev.
% Uses an iterative Newton-Raphson root-finding method.
% The maximum number of iterations before the algorithm is terminated has 
% been hardcoded in as a variable called maxiterations. It is set to 20 at 
% the start of the function, but can be changed if needed.  
% Example Inputs:
% 
% clear; clc;
% Slist = [[0; 0;  1;  4; 0;    0], ...
%        [0; 0;  0;  0; 1;    0], ...
%        [0; 0; -1; -6; 0; -0.1]];
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% T = [[0, 1, 0, -5]; [1, 0, 0, 4]; [0, 0, -1, 1.6858]; [0, 0, 0, 1]];
% thetalist0 = [1.5; 2.5; 3];
% eomg = 0.01;
% ev = 0.001;
% W = eye(6);
% [thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev, W)
% 
% Output:
% thetalist =
%    1.5707
%    2.9997
%    3.1415
% success =
%     1

thetalist = thetalist0;
i = 0;
maxiterations = 20;
Tsb = FKinSpace(M, Slist, thetalist);
Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
err = norm(Vs(1: 2)) > eomg || norm(Vs(4: 6)) > ev;
while err && i < maxiterations
    thetalist = thetalist + weighted_pinv(...
        JacobianSpace(Slist, thetalist),W) * Vs;
    i = i + 1;
    Tsb = FKinSpace(M, Slist, thetalist);
    Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    err = norm(Vs(1: 2)) > eomg || norm(Vs(4: 6)) > ev;
end
thetalist = wrapToPi(thetalist);
success = ~ err;
end

%% Wrapper fcn for Newton-Raphson inverse kinematics for a pose in task space:
function [thetalist, robot_pose, exitflag] = robot_NRIK(taskspace_pose,robot)
% "taskspace_pose" is the desired end-effector task-space configuration in SE(3)
% fprintf('IK: e-e pose in task-space frame:\n'); taskspace_pose

% express "taskspace_pose" in the robot's base frame, as "robotbase_pose"
robot_pose = robot.T_task2robot*taskspace_pose;
% fprintf('IK: e-e pose in robot base frame:\n'); robot_pose

eomg = 0.01;
ev = 0.001;
W = diag([100;100;10;1;0.1;0.01]); % weight matrix - biases NR-IK descent direction
thetalist0 = deg2rad([30;60;-30;0;60;90]);
[thetalist, success] = IKinSpaceFreewz(robot.Slist(1:robot.nJoints,:)', ...
    robot.M{robot.nJoints+1}, robot_pose, thetalist0(1:robot.nJoints),...
    eomg, ev, W);
exitflag = success;

end

%% Compute forward kinematics for a set of joint positions:
function [robot] = robot_FK(robot)
    % Calculate FK of each joint, given robot.thetalist:
    for i = 1:robot.nJoints
       T{i} = FKinSpace(robot.M{i}, robot.Slist(1:i,:)', robot.thetalist(1:i));
    end

    % Calculate FK of end-effector, given thetalist:
    T{robot.nJoints + 1} = FKinSpace(robot.M{robot.nJoints + 1}, ...
        robot.Slist(1:robot.nJoints,:)', ...
        robot.thetalist(1:robot.nJoints));
%     fprintf('FK: e-e pose in robot base frame:\n');T{robot.nJoints + 1}

    % express robot frames in task space (rotate by 90 degrees about z-axis):
    for i = 1:robot.nJoints + 1
       T{i} = robot.T_robot2task*T{i};
    end
%     fprintf('FK: e-e pose in task-space frame:\n');T{robot.nJoints + 1}

    for i = 1:robot.nJoints+1
        robot.frames.joint{i}.x = T{i}(1,4);
        robot.frames.joint{i}.y = T{i}(2,4);
        robot.frames.joint{i}.z = T{i}(3,4);
    end    
end

%% Generate a point cloud approximation of the workpiece:
function [workpiece] = gen_pts(workpiece)
    % based on example 'stldemo.m' that comes with 'stlread.m' from MATLAB
    % FileExchange - author is Eric Johnson
    workpiece.fv = stlread('cylwall.stl'); % "fv" is a struct with fields "faces" and "Vertices"
    
    % sort points vertically
    workpiece.fv.sorted_vertices = sortrows(workpiece.fv.vertices,3);
%     workpiece.fv.vertices = sortrows(workpiece.fv.vertices,3);
    
    % TODO: use a slicer, maybe
    % https://www.mathworks.com/matlabcentral/fileexchange/62113-slice_stl_create_path-triangles-slice_height
    
    workpiece.points.x = workpiece.fv.vertices(:,1) + workpiece.frame.x;
    workpiece.points.y = workpiece.fv.vertices(:,2) + workpiece.frame.y;
    workpiece.points.z = workpiece.fv.vertices(:,3) + workpiece.frame.z;

end

%% Initialize the robot
function [robot] = init_robot(robot)
% Warning: cannot be called before init_workspace() is called.
%
% Takes the initialized "robot" struct generated by init_workspace() and
% adds fields for link dimensions and the pose of every joint frame.

% dimensions (meters)
A = 0.2;
B = 0.532;
C = 0.633;
D = 2.276;
E = 1.125;
F = 1.873;
G = 1.3925;

% dimensions of prismatic end-effector:
H = 1.35;   % length of stationary part
I = 0.2;    % offset between stationary part and mobile part
J = 1.35;   % length of mobile part

% robot.extruder.stroke.max = 0;
% robot.extruder.stroke.min = -J;

shoulder_x = 0.377;
shoulder_y = 0;
shoulder_z = 0.780;

elbow_offset = 0.2;

% calculate screw axes for each joint, starting with the rotation triplet
% (omega):
omega = [0,0,1;
         0,1,0;
         0,1,0;
         1,0,0;
         0,1,0;
         1,0,0;
         0,0,0]; % joint 7 is prismatic, all others are revolute

% points used to calculate the linear triplet of each screw axis 6-vector:
q = [0,0,0;
     shoulder_x,shoulder_y,shoulder_z;
     shoulder_x,shoulder_y,shoulder_z + E;
     shoulder_x,shoulder_y,shoulder_z + E + elbow_offset;
     shoulder_x + G,shoulder_y,shoulder_z + E + elbow_offset;
     shoulder_x,shoulder_y,shoulder_z + E + elbow_offset;
     0,0,0]; % q7 is a dummy point
 
% use the points "q" and triplets "omega" to find the linear triple of each
% screw axis:
for i = 1:robot.nJoints
    v(i,:) = cross(-omega(i,:),q(i,:));
end
% v(robot.nJoints,:) = [0,0,1];

% finally, the screw axes in space frame:
for i = 1:robot.nJoints
    robot.Slist(i,:) = [omega(i,:), v(i,:)];
end

% M{i} is the SE(3) configuration of the ith joint in the robot's home position

M{1} = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
M{2} = [0,1,0,shoulder_x;
        0,0,1,shoulder_y;
        1,0,0,shoulder_z;
        0,0,0,1];
M{3} = [0,1,0,shoulder_x;
        0,0,1,shoulder_y;
        1,0,0,shoulder_z + E;
        0,0,0,1];
M{4} = [0,0,1,shoulder_x;
        0,1,0,shoulder_y;
        -1,0,0,shoulder_z + E + elbow_offset; ...
        0,0,0,1];
M{5} = [1,0,0,shoulder_x + G;
        0,0,1,shoulder_y;
        0,-1,0,shoulder_z + E + elbow_offset;
        0,0,0,1];
M{6} = [0, 0, 1, shoulder_x + G + A;
        0, 1, 0, shoulder_y;
       -1, 0, 0, shoulder_z + E + elbow_offset;
        0, 0, 0, 1];
M{7} = [0,-1, 0, shoulder_x + G + A;
        1, 0, 0, shoulder_y;
        0, 0, 1, shoulder_z + E + elbow_offset + H;
        0, 0, 0, 1];

% home configuration of the end-effector:
% M{8} = [0,-1, 0, shoulder_x + G + A + I;
%         1, 0, 0, shoulder_y;
%         0, 0, 1, shoulder_z + E + elbow_offset + H + J;
%         0, 0, 0, 1];
        
% assign M{i} to robot.M{i} for i = 1:robot.nJoints:
    for i = 1:robot.nJoints + 1
        robot.M{i} = M{i};
    end
end

%% Workspace description in task-space frame
function [perim,slab,robot,workpiece] = init_workspace(perim,slab,robot,workpiece)
% The workspace contains 4 things (represented as structs):
%   1 - workspace "perimeter"
%   2 - concrete "slab" (underneath the robot base)
%   3 - "robot" (whose base is coincident with the center of the concrete
%   slab)
%   4 - "workpiece" (the thing the robot will 3D print)

% perimeter dimensions
perim.frame.x = 0; % x-coordinate of the perimeter frame in the task-space frame
perim.frame.y = 0; % y-coordinate of the perimeter frame in the task-space frame
perim.width_dim = 4.7244;   % extends in the task-frame x-direction
perim.length_dim = 4.4196;  % extends in the task-frame y-direction
perim.height_dim = 2.5;       % extends in the task-frame z-direction (this vlaue is a guess)

% perimeter corners (A-D, clockwise, A is coincident with task-space frame)
perim.Ax = perim.frame.x + 0;
perim.Ay = perim.frame.y + 0;

perim.Bx = perim.frame.x + 0;
perim.By = perim.frame.y + perim.length_dim;

perim.Cx = perim.frame.x + perim.width_dim;
perim.Cy = perim.frame.y + perim.length_dim;

perim.Dx = perim.frame.x + perim.width_dim;
perim.Dy = perim.frame.y + 0;

perim.x_coords = [perim.Ax, perim.Bx, perim.Cx, perim.Dx];
perim.y_coords = [perim.Ay, perim.By, perim.Cy, perim.Dy];

% slab dimensions:
slab.offset.x = 2.3622;
slab.offset.y = 1.016;
slab.width_dim = 1.1176;    % extends in task-frame x-direction
slab.length_dim = 1.1176;   % extends in task-frame y-direction
slab.frame.x = slab.offset.x - 0.5*slab.width_dim; % x-coordinate of the slab frame in the task-space frame
slab.frame.y = slab.offset.y - 0.5*slab.length_dim; % y-coordinate of the slab frame in the task-space frame

% slab corners (A-D, clockwise, A is coincident with task-space frame)
slab.Ax = slab.frame.x + 0;
slab.Ay = slab.frame.y + 0;

slab.Bx = slab.frame.x + 0;
slab.By = slab.frame.y + slab.length_dim;

slab.Cx = slab.frame.x + slab.width_dim;
slab.Cy = slab.frame.y + slab.length_dim;

slab.Dx = slab.frame.x + slab.width_dim;
slab.Dy = slab.frame.y + 0;

slab.x_coords = [slab.Ax, slab.Bx, slab.Cx, slab.Dx];
slab.y_coords = [slab.Ay, slab.By, slab.Cy, slab.Dy];

% robot workspace dimensions (subsequent code will generate the actual
% robot frames and links)
robot.frames.base.x = slab.frame.x + 0.5*slab.width_dim;
robot.frames.base.y = slab.frame.y + 0.5*slab.length_dim;
robot.frames.base.z = 0;

% transformation from the robot base frame to the task-space frame:
robot.T_robot2task = [0, -1, 0, robot.frames.base.x;
                    1, 0, 0, robot.frames.base.y;
                    0, 0, 1, robot.frames.base.z;
                    0, 0, 0, 1];
% fprintf('Init: transform from robot base frame to task-space frame:\n'); robot.T_robot2task
            
% transformation from task-space frame to robot base frame:
robot.T_task2robot = TransInv(robot.T_robot2task);
% fprintf('Init: transform from task-space frame to robot base frame:\n'); robot.T_task2robot

robot.reach.min_reach = 0.994; % minimum reachable distance from robot base frame
robot.reach.max_reach = 2.848; % maximum reachable distance from robot base frame
robot.reach.min_reach_circle.x = robot.frames.base.x + ...
    robot.reach.min_reach*cos(linspace(0,2*pi,101));
robot.reach.min_reach_circle.y = robot.frames.base.y + ...
    robot.reach.min_reach*sin(linspace(0,2*pi,101));
robot.reach.max_reach_circle.x = robot.frames.base.x + ...
    robot.reach.max_reach*cos(linspace(0,2*pi,101));
robot.reach.max_reach_circle.y = robot.frames.base.y + ...
    robot.reach.max_reach*sin(linspace(0,2*pi,101));

% default "thetalist"s for robot
robot.angles.joints_home = [0;0;0;0;0;0;-1.35];
robot.angles.joints_max_ext = deg2rad([0;60;-30;0;60;0;-1.35]);
robot.angles.joints_min_ext = deg2rad([0;0;-30;0;0;0;-1.35]);
robot.angles.joints_max = deg2rad([170;85;70;300;130;360]);
robot.angles.joints_min = deg2rad([-170;-65;-180;-300;-130;-360]);

% workpiece (3D printed cylindrical wall) dimensions
workpiece.offset.x = 0;
workpiece.offset.y = 1;
workpiece.radii.outer_radius = 1.1;
workpiece.radii.inner_radius = 1.0;
workpiece.frame.x = robot.frames.base.x + workpiece.radii.outer_radius + workpiece.offset.x;
workpiece.frame.y = robot.frames.base.y + workpiece.radii.outer_radius + workpiece.offset.y;
workpiece.frame.z = 0;
workpiece.outline.x = workpiece.frame.x + workpiece.radii.outer_radius*sin(linspace(0,2*pi,101));
workpiece.outline.y = workpiece.frame.y + workpiece.radii.outer_radius*cos(linspace(0,2*pi,101));

end

%% Visualize workspace
function visualize_workspace(perim,slab,robot,workpiece)
figure;
% plot workspace perimeter:
plot3(horzcat(perim.x_coords,perim.Ax),horzcat(perim.y_coords,perim.Ay),...
    zeros(1,length(perim.x_coords)+1),'k-');
hold on;

% plot slab:
plot3(horzcat(slab.x_coords,slab.Ax),horzcat(slab.y_coords,slab.Ay),...
    zeros(1,length(slab.x_coords)+1),'k-.');

% plot workpiece:
plot3(workpiece.frame.x,workpiece.frame.y,0,'b.','MarkerSize',20)
plot3(workpiece.points.x,workpiece.points.y,workpiece.points.z,'b.:','MarkerSize',1)

% plot robot base, max & min reach:
plot3(robot.frames.base.x,robot.frames.base.y,0,'k.','MarkerSize',20);
plot3(robot.reach.min_reach_circle.x,robot.reach.min_reach_circle.y,...
    zeros(1,length(robot.reach.min_reach_circle.x)),'k:')
plot3(robot.reach.max_reach_circle.x,robot.reach.max_reach_circle.y,...
    zeros(1,length(robot.reach.max_reach_circle.x)),'k--')

% plot robot:
[robot] = robot_FK(robot);

cc = lines(robot.nJoints + 1);
joint_marker_size = 50;
link_line_width = 10;
plot3(robot.frames.joint{1}.x,robot.frames.joint{1}.y,robot.frames.joint{1}.z,...
    '.','Color',cc(1,:),'MarkerSize',joint_marker_size);
for i = 2:robot.nJoints + 1
   plot3(robot.frames.joint{i}.x,robot.frames.joint{i}.y,...
    robot.frames.joint{i}.z,'.','Color',cc(i,:),...
       'MarkerSize',joint_marker_size);
   line([robot.frames.joint{i-1}.x,robot.frames.joint{i}.x],...
       [robot.frames.joint{i-1}.y,robot.frames.joint{i}.y],...
       [robot.frames.joint{i-1}.z,robot.frames.joint{i}.z],...
       'Color',cc(i,:),'LineWidth',link_line_width);
end

axis([-1, perim.width_dim + 3, -1, perim.length_dim + 3, 0, perim.height_dim]);
pbaspect([perim.width_dim, perim.length_dim, perim.height_dim]/perim.width_dim)

legend('workspace boundary','slab',...
    'workpiece: center','workpiece: outer wall',...
    'robot: base','robot: min reach',...
    'robot: max reach','Location','Best');
% legend('workspace boundary','slab',...
%     'robot: base','robot: min reach',...
%     'robot: max reach','Location','Best');
view(80,30); % initialize viewing angle (azimuth, elevation)
% view(0,90);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
end
