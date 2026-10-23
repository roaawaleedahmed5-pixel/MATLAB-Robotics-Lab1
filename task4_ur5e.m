%% Task 4 - Jacobian singularities and damping
clear; clc; close all;
ur5e = loadrobot('universalUR5e','DataFormat','column');
showdetails(ur5e);
ee = 'tool0';
assert(ismember(ee,ur5e.BodyNames),'Inspect showdetails and set the actual tool frame name.');
qReference = [0.3; -0.8; 1.0; -0.6; 0.7; 0.2];
limits = armJointLimits(ur5e);
assert(size(limits,1)==6);
q3Range = limits(3,:); q5Range = limits(5,:);
if any(~isfinite(q3Range)), q3Range = [-2*pi 2*pi]; end
if any(~isfinite(q5Range)), q5Range = [-2*pi 2*pi]; end
q3Values = linspace(q3Range(1),q3Range(2),121);
q5Values = linspace(q5Range(1),q5Range(2),121);
[Q3,Q5] = meshgrid(q3Values,q5Values);
sigmaMin = zeros(size(Q3));
for r = 1:size(Q3,1)
    for c = 1:size(Q3,2)
        q = qReference; q(3) = Q3(r,c); q(5) = Q5(r,c);
        singularValues = svd(geometricJacobian(ur5e,q,ee));
        sigmaMin(r,c) = singularValues(end);
    end
end
figure;
surf(rad2deg(Q3),rad2deg(Q5),sigmaMin,'EdgeColor','none');
xlabel('Joint 3 (deg)'); ylabel('Joint 5 (deg)'); zlabel('Smallest singular value');
title('UR5e: joints 1, 2, 4 and 6 held fixed'); colorbar; view(45,30);

% I perturb an elbow singularity slightly so it is near-singular, not exact.
qNear = qReference; qNear(3) = 1e-3;
J = geometricJacobian(ur5e,qNear,ee);
[U,S,~] = svd(J);
% Select a fixed straight-line direction that excites the weak direction.
direction = U(4:6,end);
if norm(direction)<1e-8, direction = [1;0;0]; end
direction = direction/norm(direction);
twist = [zeros(3,1); 0.05*direction]; % [omega_x,y,z; v_x,y,z]
qdotPinv = pinv(J)*twist;
lambda = 0.02; % A lab tuning choice; record this value in the comparison.
qdotDLS = J.'*((J*J.'+lambda^2*eye(6))\twist);

fprintf('Near-singular q, rad:\n'); disp(qNear.');
fprintf('Smallest singular value: %.4e\n',S(end,end));
fprintf('Requested base-frame straight-line direction:\n'); disp(direction.');
disp(table((1:6).',qdotPinv,qdotDLS, ...
    'VariableNames',{'Joint','Pseudoinverse_rad_s','DLS_rad_s'}));
fprintf('Pseudoinverse twist residual: %.4e\n',norm(J*qdotPinv-twist));
fprintf('DLS twist residual: %.4e\n',norm(J*qdotDLS-twist));
fprintf('Achieved pinv linear velocity, m/s:\n'); disp((J(4:6,:)*qdotPinv).');
fprintf('Achieved DLS linear velocity, m/s:\n'); disp((J(4:6,:)*qdotDLS).');
fprintf('Achieved DLS angular velocity, rad/s:\n'); disp((J(1:3,:)*qdotDLS).');
% Smaller joint speeds come at the cost of a nonzero task velocity error.
