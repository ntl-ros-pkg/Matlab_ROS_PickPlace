function managePlannerRequests(q,ROS_master_ip,numPoints)
% Author: Carlos Santacruz-Rosero, Ph.D. and Tohru Kikawada
% Copyright 2016-2019 The MathWorks, Inc.

% Simulink model name
modelName = 'motominiPlanning';

% Create rigid body tree representation
motomini = createMotomini();
EELinkName = 'gripper_EE_link';

MotominiParams = setMotominiParameters(ROS_master_ip,motomini);
numJoints = MotominiParams.numJoints;

% Initialize variable to detect new ROS messages based on callback
isNewMsg = false;
% Create ROS subscriber to process planner requests
pathPlanNode = robotics.ros.Node('/matlab_path_planner_node',MotominiParams.ROS_IP);
reqSub = robotics.ros.Subscriber(pathPlanNode,'/planner/request', 'sensor_msgs/JointState');
reqSub.NewMessageFcn = @detectNewMessage;
    
% Create ROS publisher to send response
[plannerTrajPub] = robotics.ros.Publisher(pathPlanNode,'/planner/joint_trajectory',...
    'trajectory_msgs/JointTrajectory');

send(q,'Starting Planner Server');

% Start state machine
rate = robotics.Rate(2);
state = 0;
send(q,'Waiting for new ROS message')
while true 
    switch state
        case 0   % wait for new message
            if isNewMsg
                isNewMsg = false;
                state = 1;            
            else
                waitfor(rate);
            end
        case 1  % process new message
            send(q,'New ROS message received');
            objPos = [reqSub.LatestMessage.Position(1);...
                      reqSub.LatestMessage.Position(2);...
                      reqSub.LatestMessage.Position(3)];
            objID =  round(reqSub.LatestMessage.Position(4));
            qAct = zeros(numJoints,1);
            for ii=1:numJoints
                qAct(ii) = reqSub.LatestMessage.Position(4+ii);
            end 
            disp(objPos);
            disp(objID);
            disp(qAct);
            % Optimization parameters
            doOptim = false;
            totalTime = 2;
            obstacles = [];
            useParallel = false;
            showMultibodyExplorer = false;
            q0 = qAct;
            Xf.p = objPos;
            Xf.ezyx = [pi; pi; 0];
            
            % Create inverse kinematics solver and compute IK for tasks
            send(q,'Solving IK');
            [qf, info] = solveIK(motomini,EELinkName, Xf.p, Xf.ezyx, [0.1 0.1 -2 0.1 0.1 0 0 0]);
            if ~strcmp(info.Status, 'success')
                warning('Goal Task with no exact solution');
            end
            send(q,'Solving optimization');
            [QOpt,Q0,t,Qout,dQout] = planTrajectory(doOptim, q0, qf, totalTime,...
                numPoints, obstacles, MotominiParams, modelName, ...
                useParallel,showMultibodyExplorer);
            send(q,'Finished optimization');
            %
            % Process variables for response
            t = t;
            Q = Qout;
            dQ = [dQout(:,1:end-1),zeros(MotominiParams.numJoints,1)];
            % Create ROS Joint trajectory message
            jntNames = MotominiParams.jntNames;
            plannerTrajMsg = splineDataToJointTrajectoryROSMessage(Q,dQ,t,jntNames);
            % Send trajectory to Simulink model
            send(plannerTrajPub, plannerTrajMsg);
            send(q,'Trajectory sent to main controller');
            
            % set back to receive states
            state = 0;        
            send(q,'Waiting for new ROS message')
    end
end



    function [] = detectNewMessage(~, ~)
        isNewMsg = true;        
    end

end
