%% Task 5 - Cartesian pick-and-place with warm starts and null-space bias
clear; clc; close all;
[panda,qReady,ee] = setupPanda();
limits = armJointLimits(panda);
Tready = getTransform(panda,qReady,ee);
% These offsets are in the robot BASE frame; sideways is chosen as +y.
offsets = [0 0 0; 0 0 -0.15; 0 0.20 -0.15; 0 0.20 0].';
waypoints = repmat(Tready,1,1,4);
for k = 1:4
    waypoints(1:3,4,k) = Tready(1:3,4)+offsets(:,k);
end

samplesPerSegment = 201;
[s,sd,sdd,u] = trapveltraj([0 1],samplesPerSegment, ...
    'EndTime',1,'AccelTime',0.25);
% LM is intended for nearby poses solved sequentially with warm starts.
ik = inverseKinematics('RigidBodyTree',panda, ...
    'SolverAlgorithm','LevenbergMarquardt');
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.MaxIterations = 500;
ik.SolverParameters.SolutionTolerance = 1e-7;
ik.SolverParameters.GradientTolerance = 1e-12;
ik.SolverParameters.StepTolerance = 1e-12;
if isfield(ik.SolverParameters,'ErrorChangeTolerance')
    ik.SolverParameters.ErrorChangeTolerance = 1e-12;
end
Qplain = cell(1,3); Qbiased = cell(1,3); Tunit = cell(1,3);
seedPlain = qReady; seedBiased = qReady;
acceptedNonSuccessCalls = 0;

for seg = 1:3
    Tunit{seg} = transformtraj(waypoints(:,:,seg),waypoints(:,:,seg+1), ...
        [0 1],u,'TimeScaling',[s;sd;sdd]);
    A = zeros(7,samplesPerSegment); B = A;
    for k = 1:samplesPerSegment
        Tg = Tunit{seg}(:,:,k);
        if k == 1
            % Segment boundaries retain the previous endpoint exactly.
            qa = seedPlain; qb = seedBiased;
        else
            [qa,infoA] = solveVerifiedPose(ik,panda,ee,Tg,seedPlain,limits,seg,k,'ordinary');
            [qb,infoB] = solveVerifiedPose(ik,panda,ee,Tg,seedBiased,limits,seg,k,'biased');
            % A non-success flag is retained in the diagnostic count. It does
            % not invalidate a pose that independently meets the pose tolerances.
            acceptedNonSuccessCalls = acceptedNonSuccessCalls + ...
                ~strcmp(infoA.Status,'success') + ~strcmp(infoB.Status,'success');
            % A small projected gradient step tries to reduce 0.5*q1^2.
            % IK then projects the finite step back onto the requested hand pose.
            for iteration = 1:4
                J = geometricJacobian(panda,qb,ee);
                N = eye(7)-pinv(J)*J;
                step = -0.2*N(:,1)*qb(1);
                if norm(step,Inf) < 1e-9, break; end
                step = step*min(1,0.05/max(abs(step)));
                trialSeed = qb+step;
                if any(trialSeed < limits(:,1) | trialSeed > limits(:,2)), break; end
                [trial,~] = ik(ee,Tg,ones(1,6),trialSeed);
                if all(isfinite(trial)) && abs(trial(1)) < abs(qb(1)) && ...
                        all(trial >= limits(:,1) & trial <= limits(:,2)) && ...
                        norm(trial-qb,Inf) < 0.1 && poseMatches(panda,trial,ee,Tg)
                    qb = trial;
                else
                    break;
                end
            end
        end
        assert(poseMatches(panda,qa,ee,Tg) && poseMatches(panda,qb,ee,Tg));
        A(:,k) = qa; B(:,k) = qb;
        seedPlain = qa; seedBiased = qb;
    end
    Qplain{seg} = A; Qbiased{seg} = B;
end

% Retiming preserves the same Cartesian samples and IK branches.
% With normalized time u, dq/dt = (dq/du)/duration.
maxJointSpeed = 1; durations = zeros(1,3);
tAll = []; Q = []; Qbaseline = []; desired = []; achieved = [];
segmentID = []; maxLineDeviation = zeros(1,3); timeOffset = 0;
for seg = 1:3
    dqduA = diff(Qplain{seg},1,2)./diff(u);
    dqduB = diff(Qbiased{seg},1,2)./diff(u);
    normalizedPeak = max([abs(dqduA(:)); abs(dqduB(:))]);
    durations(seg) = ceil(max(1,1.10*normalizedPeak/maxJointSpeed)/0.02)*0.02;
    localTime = u*durations(seg);
    Tdesired = transformtraj(waypoints(:,:,seg),waypoints(:,:,seg+1), ...
        [0 durations(seg)],localTime, ...
        'TimeScaling',[s;sd/durations(seg);sdd/durations(seg)^2]);
    P = zeros(3,samplesPerSegment); Pa = P;
    for k = 1:samplesPerSegment
        Tgot = getTransform(panda,Qbiased{seg}(:,k),ee);
        P(:,k) = Tdesired(1:3,4,k); Pa(:,k) = Tgot(1:3,4);
    end
    startP = waypoints(1:3,4,seg);
    lineDirection = waypoints(1:3,4,seg+1)-startP;
    lineDirection = lineDirection/norm(lineDirection);
    perpendicularError = (eye(3)-lineDirection*lineDirection.')*(Pa-startP);
    maxLineDeviation(seg) = max(vecnorm(perpendicularError));
    keep = 1:samplesPerSegment;
    if seg > 1, keep = 2:samplesPerSegment; end
    tAll = [tAll timeOffset+localTime(keep)]; %#ok<AGROW>
    Q = [Q Qbiased{seg}(:,keep)]; %#ok<AGROW>
    Qbaseline = [Qbaseline Qplain{seg}(:,keep)]; %#ok<AGROW>
    desired = [desired P(:,keep)]; achieved = [achieved Pa(:,keep)]; %#ok<AGROW>
    segmentID = [segmentID seg*ones(1,numel(keep))]; %#ok<AGROW>
    timeOffset = timeOffset+durations(seg);
end
Qdot = zeros(size(Q));
for j = 1:7, Qdot(j,:) = gradient(Q(j,:),tAll); end
peakSpeed = max(abs(Qdot),[],2);
intervalSpeeds = abs(diff(Q,1,2)./diff(tAll));
assert(max(intervalSpeeds(:)) <= 1+1e-8 && max(peakSpeed) <= 1+1e-8);
assert(all(all(Q >= limits(:,1)-1e-10 & Q <= limits(:,2)+1e-10)));
fprintf('Segment durations (s):\n'); disp(durations);
disp(table((1:7).',peakSpeed,'VariableNames',{'Joint','PeakSampledSpeed_rad_s'}));
fprintf('Peak sampled position error: %.3e m\n',max(vecnorm(achieved-desired)));
fprintf('Main IK calls with non-success status but verified pose: %d\n',acceptedNonSuccessCalls);
fprintf('Maximum line deviations per segment, m:\n'); disp(maxLineDeviation);
fprintf('Joint 1 RMS: ordinary IK %.4f rad; null-space bias %.4f rad\n', ...
    sqrt(mean(Qbaseline(1,:).^2)),sqrt(mean(Q(1,:).^2)));

figure; plot(tAll,Q.'); grid on; xlabel('Time (s)'); ylabel('Joint position (rad)');
legend('q1','q2','q3','q4','q5','q6','q7','Location','best');
figure; plot(tAll,Qdot.'); hold on; yline(1,'k--'); yline(-1,'k--');
grid on; xlabel('Time (s)'); ylabel('Joint velocity (rad/s)');
figure; plot3(desired(1,:),desired(2,:),desired(3,:),'k--','LineWidth',2); hold on;
plot3(achieved(1,:),achieved(2,:),achieved(3,:),'r'); axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); legend('Commanded','Achieved');
figure; plot(tAll,[Qbaseline(1,:);Q(1,:)].'); grid on;
xlabel('Time (s)'); ylabel('Joint 1 (rad)'); legend('Ordinary IK','Null-space bias');

figure; ax = axes;
for k = 1:4:numel(tAll)
    show(panda,Q(:,k),'Parent',ax,'PreservePlot',false,'Frames','off');
    hold(ax,'on');
    plot3(ax,desired(1,:),desired(2,:),desired(3,:),'k--');
    hold(ax,'off'); view(ax,40,25); axis(ax,'equal'); drawnow;
end
show(panda,Q(:,end),'Parent',ax,'PreservePlot',false,'Frames','off');
% This is a kinematic animation, not a physics or collision simulation.
% Increase samplesPerSegment and rerun to check that measured peaks converge.

function yes = poseMatches(robot,q,ee,Tgoal)
[positionError,orientationError] = poseErrors(robot,q,ee,Tgoal);
% These are separate physical tolerances: 0.01 mm and 1e-5 rad.
yes = positionError <= 1e-5 && orientationError <= 1e-5;
end

function [positionError,orientationError] = poseErrors(robot,q,ee,Tgoal)
if any(~isfinite(q))
    positionError = Inf; orientationError = Inf;
    return;
end
Tgot = getTransform(robot,q,ee);
positionError = norm(Tgot(1:3,4)-Tgoal(1:3,4));
rotationError = Tgoal(1:3,1:3)*Tgot(1:3,1:3).';
axisAngle = rotm2axang(rotationError);
orientationError = abs(axisAngle(4));
end

function [q,info] = solveVerifiedPose(ik,robot,ee,Tgoal,seed,limits,seg,k,label)
[q,info] = ik(ee,Tgoal,ones(1,6),seed);
[positionError,orientationError] = poseErrors(robot,q,ee,Tgoal);
withinLimits = all(isfinite(q)) && ...
    all(q >= limits(:,1)-1e-10 & q <= limits(:,2)+1e-10);
if ~withinLimits || positionError > 1e-5 || orientationError > 1e-5
    error('task5:IKPoseError', ...
        ['IK pose check failed at segment %d sample %d (%s).\n' ...
         'Status: %s; position error: %.3e m; orientation error: %.3e rad; ' ...
         'within joint limits: %d.\n' ...
         'Required: position <= 1e-5 m and orientation <= 1e-5 rad.'], ...
        seg,k,label,info.Status,positionError,orientationError,withinLimits);
end
end
