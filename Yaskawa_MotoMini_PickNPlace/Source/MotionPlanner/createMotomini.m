function robot = createMotomini()
% Author: Tohru Kikawada
% Copyright 2019 The MathWorks, Inc.

% Create basic structure from URDF
robot = importrobot('motomini_with_gripper.urdf');


% Add fixed joint/link to represent custom end effector
jntEE = robotics.Joint('gripper_EE_joint','fixed');
jntEE.setFixedTransform([eye(3),[0;0;0.10];[0 0 0 1]]);
linkEE = robotics.RigidBody('gripper_EE_link');
linkEE.Joint = jntEE;

robot.addBody(linkEE, 'tool0');

end