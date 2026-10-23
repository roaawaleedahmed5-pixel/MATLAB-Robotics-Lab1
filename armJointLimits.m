function limits = armJointLimits(robot)
% Body traversal order matches the serial arm configuration order used here.
limits = zeros(0,2);
for k = 1:numel(robot.Bodies)
    joint = robot.Bodies{k}.Joint;
    if ~strcmp(joint.Type,'fixed')
        limits(end+1,:) = joint.PositionLimits; %#ok<AGROW>
    end
end
end
