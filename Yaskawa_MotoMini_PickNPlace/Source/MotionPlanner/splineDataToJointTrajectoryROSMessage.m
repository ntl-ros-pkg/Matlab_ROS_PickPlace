function [msg] = splineDataToJointTrajectoryROSMessage(Q,dQ,t, jntNames)
% Author: Carlos Santacruz-Rosero, Ph.D.
% Copyright 2016 The MathWorks, Inc.

% Get size of spline matrix
[numJoints,numPoints] = size(Q);

% Create joint trajectory message
msg = rosmessage(rostype.trajectory_msgs_JointTrajectory);
msg.JointNames = jntNames;

% Create joint trajectory point messages and load them to joint trajectory
% Not preallocating, not critical here
for ii = 1:numPoints    
    jntJTP(ii) = rosmessage(rostype.trajectory_msgs_JointTrajectoryPoint);
    dur = robotics.ros.msg.Duration;
    dur.Sec = floor(t(ii));
    dur.Nsec = round(1e9 * (t(ii)-floor(t(ii))));
    jntJTP(ii).TimeFromStart = dur;
    jntJTP(ii).Positions = Q(:,ii);
    jntJTP(ii).Velocities = dQ(:,ii);
%   jntJTP(ii).Accelerations = zeros(YoubotParams.numJoints,1);    
end

msg.Points = jntJTP;

end