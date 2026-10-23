%% Run this BEFORE opening/running the Simulink model.
% Do not clear this workspace after running it: the blocks need these variables.
[panda,qReady,ee] = setupPanda();
limits = armJointLimits(panda);
Tready = getTransform(panda,qReady,ee);
Ts = 0.02;
circleRadius = 0.10; % Radius assumption: the sheet only says '10 cm circle'.
circlePeriod = 4;
Tcircle0 = Tready;
Tcircle0(1:3,4) = Tready(1:3,4)+[circleRadius;0;0];
ikInit = inverseKinematics('RigidBodyTree',panda, ...
    'SolverAlgorithm','LevenbergMarquardt');
ikInit.SolverParameters.AllowRandomRestart = false;
ikInit.SolverParameters.MaxIterations = 500;
ikInit.SolverParameters.SolutionTolerance = 1e-7;
ikInit.SolverParameters.GradientTolerance = 1e-12;
ikInit.SolverParameters.StepTolerance = 1e-12;
if isfield(ikInit.SolverParameters,'ErrorChangeTolerance')
    ikInit.SolverParameters.ErrorChangeTolerance = 1e-12;
end
[qCircle0,info0] = ikInit(ee,Tcircle0,ones(1,6),qReady);
Tinitial = getTransform(panda,qCircle0,ee);
initialPositionError = norm(Tinitial(1:3,4)-Tcircle0(1:3,4));
relativeRotation = Tcircle0(1:3,1:3)*Tinitial(1:3,1:3).';
initialAxisAngle = rotm2axang(relativeRotation);
initialOrientationError = abs(initialAxisAngle(4));
withinLimits = all(isfinite(qCircle0)) && ...
    all(qCircle0 >= limits(:,1)-1e-10 & qCircle0 <= limits(:,2)+1e-10);
% Check the actual pose and limits, independently of the solver status.
assert(withinLimits && initialPositionError <= 1e-5 && ...
    initialOrientationError <= 1e-5, ...
    ['Initial circle pose failed validation: status %s, position %.3e m, ' ...
     'orientation %.3e rad, within limits %d.'], ...
    info0.Status,initialPositionError,initialOrientationError,withinLimits);
fprintf('Initial circle IK: %s; position error %.3e m; rotation error %.3e rad\n', ...
    info0.Status,initialPositionError,initialOrientationError);
fprintf('Robot base frame: %s\n',panda.BaseName);
fprintf('Use qCircle0 as the Unit Delay initial condition, not qReady.\n');
disp(qCircle0);
