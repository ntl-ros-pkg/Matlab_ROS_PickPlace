function MotominiParams = setMotominiParameters(IPAddress,motominiModel)
% Author: Tohru Kikawada
% Copyright 2019 The MathWorks, Inc.

% Index of revolute joints
indexRevolute = cellfun(@(x) strcmp(x.Joint.Type,'revolute'),motominiModel.Bodies);

% The number of revolute joints
MotominiParams.numJoints = sum(indexRevolute);

% Get joint names
MotominiParams.jntNames = ...
    cellfun(@(x) x.Joint.Name,motominiModel.Bodies(indexRevolute),'UniformOutput',false);

% URDF
jntLimits = cell2mat(cellfun(@(x) x.Joint.PositionLimits',motominiModel.Bodies(indexRevolute),'UniformOutput',false));
MotominiParams.jntMinLim = jntLimits(1,:)';
MotominiParams.jntMaxLim = jntLimits(2,:)';
% Motomini 
MotominiParams.qHome = 1e-3*ones(MotominiParams.numJoints,1);
% Gripper parameters
MotominiParams.numFingers = 2;
MotominiParams.fingerNames = {'gripper_finger_joint_l';'gripper_finger_joint_r'};
MotominiParams.fingerMinLim = [     0;      0];
MotominiParams.fingerMaxLim = [0.0115; 0.0115];
% ROS interface parameters
MotominiParams.ROS_IP = IPAddress;


end
