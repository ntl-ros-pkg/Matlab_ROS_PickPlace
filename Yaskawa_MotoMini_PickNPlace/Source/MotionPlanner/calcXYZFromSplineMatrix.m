function XYZ = calcXYZFromSplineMatrix(robot,EELinkName, numPoints,Q)

C1 = robot.homeConfiguration;
XYZ = zeros(numPoints+2, 3);

for ii =1:(numPoints + 2)
    % Original
    C1 = setJointPosInJointConfig(C1, [Q(:,ii)',0,0]);
    T1 = getTransform(robot,C1,EELinkName);
    XYZ(ii,1:3) = T1(1:3,4)';  
end
 

end
