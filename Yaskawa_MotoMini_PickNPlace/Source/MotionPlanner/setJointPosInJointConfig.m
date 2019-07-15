function Q = setJointPosInJointConfig(Q, q)

Q = arrayfun(@(x,y) setfield(x, 'JointPosition', y), Q, q);

end