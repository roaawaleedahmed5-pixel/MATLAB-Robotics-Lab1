%% Task 3 - Workspace and analytical versus numerical IK
clear; clc; close all; rng(4);
[scara,p] = makeSCARA();

% I sample joint space and then use FK to find the reachable tool positions.
xy = zeros(2,2000);
for k = 1:2000
    q = randomConfiguration(scara);
    T = getTransform(scara,q,'tool');
    xy(:,k) = T(1:2,4);
end
figure; scatter(xy(1,:),xy(2,:),5,'.');
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
title('SCARA workspace: 2000 joint configurations');

% Four targets are made with FK so they are known to be feasible.
qKnown = [0.2 -0.4  0.7 -0.8;
          0.8 -1.0  1.2 -0.7;
          0.05 0.10 0.15 0.08;
          0.1  0.2 -0.2  0.3];
targets = zeros(5,4);
for k = 1:4
    T = getTransform(scara,qKnown(:,k),'tool');
    targets(k,:) = [T(1:3,4).' atan2(T(2,1),T(1,1))];
end
targets(5,:) = [0.90 0 0.45 0]; % Beyond the maximum 0.7 m planar reach.
disp('Targets [x y z yaw], m and rad:'); disp(targets);

ik = inverseKinematics('RigidBodyTree',scara);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.SolutionTolerance = 1e-8;
weights = ones(1,6);
seeds = [0.3 -0.3; 0.9 -0.9; 0.1 0.1; 0 0];
comparison = [];
for k = 1:5
    target = targets(k,:);
    Tgoal = trvec2tform(target(1:3))*eul2tform([target(4) 0 0],'ZYX');
    [Q,valid,reason] = scaraIK(target(1),target(2),target(3),target(4),p);
    fprintf('\nTarget %d: %s\n',k,reason);
    disp('Analytical columns: +sin(q2), -sin(q2) (rad, rad, m, rad)'); disp(Q);
    disp('Valid branches:'); disp(valid);
    for b = find(valid)
        assert(norm(getTransform(scara,Q(:,b),'tool')-Tgoal,'fro') < 1e-9);
    end
    for s = 1:2
        [qn,info] = ik('tool',Tgoal,weights,seeds(:,s));
        Tgot = getTransform(scara,qn,'tool');
        posError = norm(Tgot(1:3,4)-Tgoal(1:3,4));
        rotError = norm(Tgot(1:3,1:3)-Tgoal(1:3,1:3),'fro');
        % Compare angles modulo 2*pi, but do not wrap the prismatic coordinate.
        branchDistance = [NaN NaN];
        for b = find(valid)
            delta = qn-Q(:,b);
            delta([1 2 4]) = atan2(sin(delta([1 2 4])),cos(delta([1 2 4])));
            branchDistance(b) = norm(delta); % Diagnostic with mixed units.
        end
        fprintf('Seed %d: %s; position error %.3e m; rotation matrix error %.3e\n', ...
                s,info.Status,posError,rotError);
        disp('Numerical q:'); disp(qn.');
        disp('Distance to the two analytical branches:'); disp(branchDistance);
        comparison = [comparison; k s strcmp(info.Status,'success') posError rotError branchDistance]; %#ok<AGROW>
    end
end
results = array2table(comparison,'VariableNames', ...
    {'Target','Seed','Success','PositionError_m','RotationMatrixError','DistancePlus','DistanceMinus'});
disp(results);
