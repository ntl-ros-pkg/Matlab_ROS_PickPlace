function [QOpt,Q0,t,Qout,dQout] = planTrajectory(doOptim, q0, qf,totalTime, numPoints,obstacles,...
    RobotParams, modelName, useParallel, showMultibodyExplorer)
% Author: Carlos Santacruz-Rosero, Ph.D.
% Copyright 2016 The MathWorks, Inc.

% Index
TORQUE_IND = 1;
Q_IND = 2;
DQ_IND = 3;
DOBJ1_IND =4;
% Soft limit tolerance
SOFT_JNT_LIMIT_TOL = deg2rad(0);

%% Open Simulink Model with Fast Restart Enabled
%open_system(modelName);
load_system(modelName);
%set_param(modelName,'FastRestart','off');
set_param([modelName '/Draw Trajectory'],'Commented','on');

if isempty(obstacles)
    set_param([modelName '/Collision Model'],'Commented','on');
end

if showMultibodyExplorer
    set_param(modelName,'SimMechanicsOpenEditorOnUpdate','on'); 
else
    set_param(modelName,'SimMechanicsOpenEditorOnUpdate','off');
end
set_param(modelName,'FastRestart','on'); 

%% Create initial trajectory using spline parametrization
t0 = 0;
% Create linearly spaced time points and trajectories
t = linspace(t0,totalTime,numPoints + 2);

% Qt = |q1(t0) q1(t1) ... q1(tf);
%      [q2(t0) q2(t1) ... q2(tf);
%      ...
%      |q5(t0) q5(t1) ... q5(tf)|
Q0 = zeros(RobotParams.numJoints, numPoints + 2);
for i = 1:RobotParams.numJoints
    Q0(i,:) = linspace(q0(i), qf(i),numPoints + 2);
end
Qt = Q0;


%% Simulate model to show initial cost
MotominiParams.jntMaxLim = RobotParams.jntMaxLim;
MotominiParams.jntMinLim = RobotParams.jntMinLim;

% % Show trajectory cost
% simOut = sim(modelName,'SrcWorkspace', 'current', 'StopTime', num2str(totalTime));
% % Get torque cost
% out = simOut.get('yout');
% fCost = out.getElement(TORQUE_IND).Values.Data(end,1);
% %fCost = outputs(end,1);
% fprintf('Initial Torque Cost: %d\n', fCost);


%% Set optimization problem
% Num Optimization variables = numJoints * numViaPoints
xOld = [];  % tracks previous optimal values
yout = [];  % simulation output

% Set cost function
fun = @(x) costFunTrajectoryPlanning(x);

% Set constraint function
if isempty(obstacles)
    nonlcon = [];
else
    nonlcon = @(x) constraintFunTrajectoryPlanning(x);
end

% Create initial optimization vector from spline matrix
% Select via points from Q matrix and expand into vector
% [Q0(:,2);Q0(:,3);Q(:,4),...];
x0 = Q0(:,2:numPoints+1);
x0 = x0(:);


A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
for ii = 1:numPoints
    lb = [lb; RobotParams.jntMinLim+SOFT_JNT_LIMIT_TOL];
    ub = [ub; RobotParams.jntMaxLim-SOFT_JNT_LIMIT_TOL];
end

% Optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm',...
    'interior-point', 'PlotFcn', @optimplotfval,'MaxIterations',30);
if useParallel
    options.UseParallel = true;
    disp('Optim with Parallel Pool: Turning off Multibody Explorer');
    set_param(modelName,'SimMechanicsOpenEditorOnUpdate','off');
end

%% Solve optimization problem
if doOptim
    [xOpt, fobj, exitFlag, outputFree] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon, ...
        options);
else
    xOpt = x0;
end
% Convert optim vector to spline matrix
xOpt = reshape(xOpt, RobotParams.numJoints, numPoints);
QOpt = Q0;
QOpt(:,2:numPoints+1) = xOpt;


%% Get X and dX matrices from simulation
Qt = QOpt;
simOut = sim(modelName,'SrcWorkspace', 'current', 'StopTime', num2str(totalTime));
out = simOut.get('yout');
tout = simOut.get('tout');
q = out.getElement(Q_IND).Values.Data(:,:);
dq = out.getElement(DQ_IND).Values.Data(:,:);
tInd = ismembertol(tout,t);
tNew = tout(tInd);
if length(tNew) ~= length(t)
    error('planTrajectory: Time vectors don''t have the same length');
end
Qout = q(tInd,:)';
dQout = dq(tInd,:)';

%%
set_param(modelName,'FastRestart','off');
set_param([modelName '/Draw Trajectory'],'Commented','off');


%% Objective function: minimize Torque effort
function [f] = costFunTrajectoryPlanning(x)
        updateIfNeeded(x);      
        % f = yout(end,1);      % tauCost;
        %f = yout.getElement('torqueCost').Values.Data(end,1);
        f = yout.getElement(TORQUE_IND).Values.Data(end,1);
        
end

function [c,ceq] = constraintFunTrajectoryPlanning(x)
        updateIfNeeded(x);
        %dobj1 = -1*yout(:,12);   % distObj1
        %dobj1 = -1*yout.getElement('distObj1').Values.Data(:,1);
        dobj1 = -1*yout.getElement(DOBJ1_IND).Values.Data(:,1);
        numSamples = 20;      % take only every this number of samples
        selectIndex = 1:numSamples:length(dobj1);
        c = dobj1(selectIndex);
        ceq = [];
end


%% Helper function to run simulation only when needed
% See: http://www.mathworks.com/help/optim/ug/example-using-fminimax-with-a-simulink-model.html
function updateIfNeeded(x)
     if ~isequal(x,xOld) % compute only if needed
         
        % Map optimization vector to spline matrix
        Xvp = reshape(x, RobotParams.numJoints, numPoints);
        Qt(:,2:numPoints+1) = Xvp;    

        % Run simulation
        simOut = sim(modelName, 'SrcWorkspace', 'current',...
            'StopTime', num2str(totalTime));

        % Get Torque Cost function from matrix. 
        % outputs: [torqueCost(1x1), dist2obj1(1x1)]
        yout = simOut.get('yout');
        
        xOld = x;
     end
end


end