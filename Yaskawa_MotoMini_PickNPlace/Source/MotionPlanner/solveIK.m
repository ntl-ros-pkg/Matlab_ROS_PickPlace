function [q, info] = solveIK(robot, EELinkName, p, ezyx, q0)
% Author: Carlos Santacruz-Rosero, Ph.D.
% Copyright 2016 The MathWorks, Inc.

ik = robotics.InverseKinematics('RigidBodyTree', robot);
CInitialGuess = robot.homeConfiguration;
CInitialGuess = setJointPosInJointConfig(CInitialGuess, q0);
ikWeights = [1 1 1 1 1 1];

% Calculate initial task rigid body transformation
Tee= eul2tform(ezyx');
Tee(1:3,4) = p;

% Solve inverse kinematics for the two tasks
[C, info] = step(ik, EELinkName, Tee, ikWeights, CInitialGuess);
q = [C.JointPosition];


end