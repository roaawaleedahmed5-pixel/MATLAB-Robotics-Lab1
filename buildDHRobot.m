function robot = buildDHRobot(DH, jointTypes, limits)

n = size(DH,1);
assert(size(DH,2) == 4 && isequal(size(limits),[n 2]));
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',n+1);
parent = robot.BaseName;
for i = 1:n
    body = rigidBody(sprintf('link%d',i));
    joint = rigidBodyJoint(sprintf('joint%d',i),jointTypes{i});
    joint.JointAxis = [0 0 1];
    setFixedTransform(joint,DH(i,:),'dh');
    joint.PositionLimits = limits(i,:);
    joint.HomePosition = min(max(0,limits(i,1)),limits(i,2));
    body.Joint = joint;
    addBody(robot,body,parent);
    parent = body.Name;
end
end
