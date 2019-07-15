% This script shows how to calculate the trajectory Home To Idle pose
% using an optimization-based trajectory planner. This is a complex
% robotics problem that we solve using MATLAB, Simulink, Robotics System
% Toolbox, Simscape Multibody, and Optimization Toolbox.
% Intented for offline trajectory planning.
% Author: Carlos Santacruz-Rosero, Ph.D.
% Copyright 2016 The MathWorks, Inc.


%% Simulink model name
modelName = 'youBotPlanning';


%% Create rigid body tree representation
youbot = createYoubot();
EELinkName = 'gripper_EE_link';


%% YoubotParams
YoubotParams = setYouBotParameters('');


%% Planning Parameters
doOptim = true;
% Initial task (starting joint positions)
% X0.p = [0.0437;-0.0055;0.3976]; 
% X0.ezyx = [-0.6043;  0.2611; -0.0881];
q0 = YoubotParams.qHome;
% Final task (to idle pose)
Xf.p =  [0.2;0;0.3]; 
Xf.ezyx = [0; pi/2; 0];
% Planning time
totalTime = 2;
% Collision objects
obstacles = [];
% Define three via points
numPoints = 4;
% Parallel computing
useParallel = false;
% Visualization
showMultibodyExplorer = true;



%% Create inverse kinematics solver and compute IK for tasks
[qf, info] = solveIK(youbot,EELinkName, Xf.p, Xf.ezyx, [0.1 0.1 -2 0.1 0.1 0 0]);
if ~strcmp(info.Status, 'success')
    warning('Goal Task with no exact solution');
end

%% Define obstacle
set_param([modelName '/Collision Model'],'Commented','off')
colObject1.type = 'sphere';
colObject1.radius = 0.13;
colObject1.pos = [0.; 0 ; 0];
obstacles = [colObject1];   % in the future this will be an array. 


%% Solve planning problem with no obstacles
[QOpt,Q0,t,Qout,dQout] = planTrajectory(doOptim, q0, qf, totalTime, numPoints, obstacles,...
    YoubotParams, modelName, useParallel,showMultibodyExplorer);


%% Save data for demo
QOptObs = QOpt;
Q0Obs = Q0;
tObs = t;
QoutObs = Qout;
dQoutObs = dQout;
save('YBOptimResultsForDemoShowcase.mat','QOptObs','Q0Obs','tObs',...
     'QoutObs','dQoutObs', '-append'); 
 
 
%% Load optimization results Home To Idle
load('YBOptimResultsForDemoShowcase.mat')
%% No obstacle
QOpt = QOptNoObs;
Q0 = Q0NoObs;
t = tNoObs;
Qout = QoutNoObs;
dQout = dQoutNoObs;
%% Obstacle
QOpt = QOptObs;
Q0 = Q0Obs;
t = tObs;
Qout = QoutObs;
dQout = dQoutObs;


%% Post process data:
Qt = QOpt;
% Create splines in XYZ to visualize trajectory
q0 = Q0(:,1);     % Required for initial value of joints
set_param(modelName,'SimMechanicsOpenEditorOnUpdate','on');
XYZOr = calcXYZFromSplineMatrix(youbot,EELinkName,numPoints,Q0);
XYZOpt = calcXYZFromSplineMatrix(youbot,EELinkName,numPoints,QOpt);
% Process variables for response
homeToIdleSpline.t = t;
homeToIdleSpline.Q = Qout;
homeToIdleSpline.dQ = [dQout(:,1:end-1),zeros(YoubotParams.numJoints,1)]; 
% save('youbotSplineData.mat', 'homeToIdleSpline');


%% Now calculate Idle To Home Trajectory
q0Aux = q0;
q0 = qf;
qf = q0Aux;
[QOpt,Q0,t,Qout,dQout] = planTrajectory(q0, qf, totalTime, numPoints, obstacles,...
    YoubotParams, modelName, useParallel,showMultibodyExplorer);


%% Post process data:
Qt = QOpt;
% Create splines in XYZ to visualize trajectory
% q0 = Q0(:,1);     % Required for initial value of joints
set_param(modelName,'SimMechanicsOpenEditorOnUpdate','on');
XYZOr = calcXYZFromSplineMatrix(youbot,EELinkName,numPoints,Q0);
XYZOpt = calcXYZFromSplineMatrix(youbot,EELinkName,numPoints,QOpt);
% Process variables for response
idleToHomeSpline.t = t;
idleToHomeSpline.Q = Qout;
idleToHomeSpline.dQ = [dQout(:,1:end-1),zeros(YoubotParams.numJoints,1)]; 
%save('youbotSplineData.mat', 'idleToHomeSpline', '-append');


