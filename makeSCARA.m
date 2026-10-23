function [robot, p] = makeSCARA()
% I chose all joint z axes upward, so positive q3 raises the tool.
% The base height and stroke below are design assumptions, not sheet data.
p.L1 = 0.4; p.L2 = 0.3; p.h = 0.4; p.toolDrop = 0.05;
p.limits = [deg2rad([-170 170]); deg2rad([-150 150]); ...
            0 0.20; deg2rad([-180 180])];
p.DH = [p.L1 0 p.h 0;
        p.L2 0 0   0;
        0    0 0   0;
        0    0 0   0];
robot = buildDHRobot(p.DH,{'revolute','revolute','prismatic','revolute'},p.limits);
tool = rigidBody('tool');
joint = rigidBodyJoint('tool_fixed','fixed');
setFixedTransform(joint,trvec2tform([0 0 -p.toolDrop]));
tool.Joint = joint;
addBody(robot,tool,'link4');
end
