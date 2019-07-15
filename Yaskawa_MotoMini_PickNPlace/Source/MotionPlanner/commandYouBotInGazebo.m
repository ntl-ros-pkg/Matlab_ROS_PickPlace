% Author: Carlos Santacruz-Rosero, Ph.D.
% Copyright 2016 The MathWorks, Inc.



%% Robot Parameters
YoubotParams = setYouBotParameters('192.168.204.202');


%% Connect to ROS
rosshutdown;
rosinit(YoubotParams.ROS_IP);


%%
% qAct = [0.0411; 0.0360; -0.2189; 0.0233; 0.0240];
qHome = [0; 0; 0; 0; 0.05];
qCube = [2.9671;    1.7376;   -1.4119;    3.1911;    2.93]; % 2.93


%% Get state from robot
armStateSub = rossubscriber('/arm_1/arm_controller/state');
pause(1)
qAct = armStateSub.LatestMessage.Actual.Positions
display(armStateSub.LatestMessage.JointNames);


%% Create publisher and message to send command to robot
[armCmdPub,armCmdMsg] = rospublisher('/arm_1/arm_controller/command');
display(armCmdMsg);    % 'trajectory_msgs/JointTrajectory'


%% Create Joint Trajectory Point messages
% Joint Trajectory Point 1: all joints to zero
jntJTP1 = rosmessage(rostype.trajectory_msgs_JointTrajectoryPoint);
dur1 = robotics.ros.msg.Duration;
dur1.Sec = 0;
jntJTP1.TimeFromStart = dur1;
jntJTP1.Positions = qHome;
jntJTP1.Velocities = zeros(YoubotParams.numJoints,1);
jntJTP1.Accelerations = zeros(YoubotParams.numJoints,1);
% Joint Trajectory Point 2:
jntJTP2 = rosmessage(rostype.trajectory_msgs_JointTrajectoryPoint);
dur2 = robotics.ros.msg.Duration;
dur2.Sec = 2.0;
jntJTP2.TimeFromStart = dur2;
jntJTP2.Positions = zeros(YoubotParams.numJoints,1);
% jntJTP2.Positions(1) = pi/2;
% jntJTP2.Positions(2) = pi/2;
% jntJTP2.Positions(3) = deg2rad(-180);
% jntJTP2.Positions(4) = deg2rad(45);
jntJTP2.Positions = [1.5255    1.3542   -1.3738    3.5364    1.4819];
jntJTP2.Velocities = zeros(YoubotParams.numJoints,1);
jntJTP2.Accelerations = zeros(YoubotParams.numJoints,1);


%% Configure command message 'trajectory_msgs/JointTrajectory' and send
armCmdMsg.JointNames = YoubotParams.jntNames;
armCmdMsg.Points = [jntJTP1, jntJTP2];
armCmdMsg.Points = [jntJTP2];
send(armCmdPub, armCmdMsg);


%% Check state of gripper controller
gripperStateSub = rossubscriber('/arm_1/gripper_controller/state');
pause(1);
gripperAct = gripperStateSub.LatestMessage.Actual.Positions
display(gripperStateSub.LatestMessage.JointNames);


%% Still figuring out how the gripper moves
[gripperCmdPub, gripperCmdMsg] = rospublisher('/arm_1/gripper_controller/command');
% Joint Trajectory Point
gripperJTP1 = rosmessage(rostype.trajectory_msgs_JointTrajectoryPoint);
dur1 = robotics.ros.msg.Duration;
dur1.Sec = 1;
dur1.Nsec = 0;
gripperJTP1.TimeFromStart = dur1;
gripperJTP1.Positions = zeros(YoubotParams.numFingers,1);
gripperJTP1.Velocities = zeros(YoubotParams.numFingers,1);
gripperJTP1.Accelerations = zeros(YoubotParams.numFingers,1);
gripperJTP1.Positions(1) = 0.025;
gripperJTP1.Positions(2) = 0.025;

%%
gripperJTP1.Positions(1) = 0;  
gripperJTP1.Positions(2) = 0;

%% Send Trajectory to Gripper
gripperCmdMsg.JointNames = YoubotParams.fingerNames;
gripperCmdMsg.Points = [gripperJTP1];
send(gripperCmdPub, gripperCmdMsg);




%% Test grabbing gripper

%% Get state from robot
rosshutdown;
rosinit(ROSMasterIP);
[armCmdPub,armCmdMsg] = rospublisher('/arm_1/arm_controller/command');
[gripperCmdPub, gripperCmdMsg] = rospublisher('/arm_1/gripper_controller/command');
jntJTP1 = rosmessage(rostype.trajectory_msgs_JointTrajectoryPoint);
dur1 = robotics.ros.msg.Duration;
dur1.Sec = 2;
jntJTP1.TimeFromStart = dur1;
jntJTP1.Positions = zeros(YoubotParams.numJoints,1);
jntJTP1.Velocities = zeros(YoubotParams.numJoints,1);
jntJTP1.Accelerations = zeros(YoubotParams.numJoints,1);
% Gripper open
gripperJTP1 = rosmessage(rostype.trajectory_msgs_JointTrajectoryPoint);
dur1 = robotics.ros.msg.Duration;
dur1.Sec = 1;
dur1.Nsec = 0;
gripperJTP1.TimeFromStart = dur1;
gripperJTP1.Positions = [0.025; 0.025];
gripperJTP1.Velocities = zeros(YoubotParams.numFingers,1);
gripperJTP1.Accelerations = zeros(YoubotParams.numFingers,1);

%% Aproaching point
jntJTP1.Positions = [2.5287    1.9489   -1.7816    3.3496    2.4851];

%% Grabbing point
jntJTP1.Positions = [2.5287    1.9959   -1.5691    3.0901    2.4851];

%% Dropping Points
jntJTP1.Positions = [1.5255    1.3542   -1.3738    3.5364    1.4819];

%% Send trajectory
armCmdMsg.JointNames = YoubotParams.jntNames;
armCmdMsg.Points = [jntJTP1];
send(armCmdPub, armCmdMsg);

%% Gripper open
gripperJTP1 = rosmessage(rostype.trajectory_msgs_JointTrajectoryPoint);
dur1 = robotics.ros.msg.Duration;
dur1.Sec = 1;
dur1.Nsec = 0;
gripperJTP1.TimeFromStart = dur1;
gripperJTP1.Positions = zeros(YoubotParams.numFingers,1);
gripperJTP1.Velocities = zeros(YoubotParams.numFingers,1);
gripperJTP1.Accelerations = zeros(YoubotParams.numFingers,1);

%% Gripper open
gripperJTP1.Positions(1) = 0.025;
gripperJTP1.Positions(2) = 0.025;

%% Gripper close
gripperJTP1.Positions(1) = 0.0;
gripperJTP1.Positions(2) = 0.0;


%% Send Trajectory to Gripper
gripperCmdMsg.JointNames = YoubotParams.fingerNames;
gripperCmdMsg.Points = [gripperJTP1];
send(gripperCmdPub, gripperCmdMsg);


