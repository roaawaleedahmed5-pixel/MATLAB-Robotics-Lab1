function [summary,jointResults] = run_task7(runSimulations,stopTime)
% Task 7: prepare the supplied TA model and run three poses with B = 0 and 1.
% Usage: run_task7                 prepares and runs all six experiments
%        run_task7(false)          prepares the model for visual inspection
%        run_task7(true,40)        repeats with a longer observation interval
% Requires MATLAB, Simulink, Simscape and Simscape Multibody on your computer.
% This script has been checked against the supplied model XML, but has not
% been executed in MATLAB by the report authoring environment.
if nargin < 1, runSimulations = true; end
if nargin < 2, stopTime = 20; end
validateattributes(stopTime,{'double'},{'scalar','finite','>=',2});
rootFolder = fileparts(mfilename('fullpath'));
sourceFile = fullfile(rootFolder,'panda_multibody.slx');
meshFolder = fullfile(rootFolder,'panda_meshes');
assert(isfile(sourceFile),'Keep panda_multibody.slx beside this script.');
assert(isfolder(meshFolder),'Keep the supplied panda_meshes folder beside this script.');
stamp = datestr(now,'yyyymmdd_HHMMSS');
outputFolder = fullfile(rootFolder,'task7_results',stamp);
assert(~isfolder(outputFolder),'A run already exists for this second. Run again.');
mkdir(outputFolder);
model = ['panda_task7_' stamp];
modelFile = fullfile(outputFolder,[model '.slx']);
copyfile(sourceFile,modelFile);
load_system(modelFile);

% The geometry on the TA computer used absolute paths. Repair only the copy.
blocks = find_system(model,'LookUnderMasks','all','FollowLinks','off','Type','Block');
repairedMeshes = 0;
for k = 1:numel(blocks)
    parameters = get_param(blocks{k},'DialogParameters');
    if isfield(parameters,'ExtGeomFileName')
        oldName = strrep(get_param(blocks{k},'ExtGeomFileName'),'\','/');
        [~,name,extension] = fileparts(oldName);
        localMesh = fullfile(meshFolder,[name extension]);
        assert(isfile(localMesh),'Mesh not found: %s',localMesh);
        set_param(blocks{k},'ExtGeomFileName',localMesh);
        repairedMeshes = repairedMeshes+1;
    end
end
assert(repairedMeshes >= 7,'Expected the mesh blocks from the supplied Panda model.');

% These are the seven arm bounds already present in the supplied URDF/model.
limits = [-2.8973 2.8973; -1.7628 1.7628; -2.8973 2.8973; ...
          -3.0718 -0.0698; -2.8973 2.8973; -0.0175 3.7525; ...
          -2.8973 2.8973];
qReady = [0;-pi/4;0;-3*pi/4;0;pi/2;pi/4];
poses = [qReady, [0.4;-0.5;0.4;-1.8;0.3;1.3;0], ...
         [-0.4;0.3;-0.3;-2.2;-0.4;2.0;0.4]];
assert(all(all(poses >= limits(:,1) & poses <= limits(:,2))));
set_param([model '/MechanismConfiguration'], ...
    'GravityVector','[0 0 -9.81]');
set_param(model,'SolverType','Variable-step','Solver','ode15s', ...
    'RelTol','1e-5','AbsTol','1e-7','MaxStep','0.001', ...
    'StartTime','0','StopTime',num2str(stopTime), ...
    'ReturnWorkspaceOutputs','on');
set_param([model '/log q'],'VariableName','qArm', ...
    'SaveFormat','Timeseries','MaxDataPoints','inf','Decimation','1');

sensorOptions = {'SenseVelocity','SenseLowerLimitTorqueForce','SenseUpperLimitTorqueForce'};
prefixes = {'w','lowerTorque','upperTorque'};
units = {'rad/s','N*m','N*m'};
variables = {'wArm','lowerLimitTorque','upperLimitTorque'};
for category = 1:3
    muxName = [model '/' prefixes{category} 'Mux'];
    add_block('simulink/Signal Routing/Mux',muxName,'Inputs','7', ...
        'Position',[1870 330+category*180 1875 450+category*180]);
    sinkName = [model '/log_' variables{category}];
    add_block('simulink/Sinks/To Workspace',sinkName, ...
        'VariableName',variables{category},'SaveFormat','Timeseries', ...
        'MaxDataPoints','inf','Decimation','1', ...
        'Position',[1920 365+category*180 2050 395+category*180]);
    add_line(model,[prefixes{category} 'Mux/1'], ...
        ['log_' variables{category} '/1'],'autorouting','on');
end
stopSettings = cell(7,1);
for j = 1:7
    joint = [model '/panda_joint' num2str(j)];
    assert(getSimulinkBlockHandle(joint)>0, ...
        'Expected the supplied arm joint at %s.',joint);
    stopSettings{j} = captureStopSettings(joint);
    set_param(joint,'JointMode','Normal','TorqueActuationMode','NoTorque', ...
        'MotionActuationMode','ComputedMotion','SpringStiffness','0', ...
        'DampingCoefficient','0','DampingCoefficientUnits','kg*m^2/(rad*s)', ...
        'PositionTargetSpecify','on','PositionTargetPriority','High', ...
        'PositionTargetValue',num2str(poses(j,1),17),'PositionTargetValueUnits','rad', ...
        'VelocityTargetSpecify','on','VelocityTargetPriority','High', ...
        'VelocityTargetValue','0','VelocityTargetValueUnits','rad/s');
    assert(strcmp(get_param(joint,'LowerLimitSpecify'),'on') && ...
        strcmp(get_param(joint,'UpperLimitSpecify'),'on'),'Joint limits must be enabled.');
    assert(abs(str2double(get_param(joint,'LowerLimitBound'))-limits(j,1))<1e-10 && ...
        abs(str2double(get_param(joint,'UpperLimitBound'))-limits(j,2))<1e-10, ...
        'Unexpected limits on joint %d.',j);
    set_param([model '/q' num2str(j)],'Unit','rad');
    for category = 1:3
        % Expose one new sensor at a time. All existing right-side ports are
        % connected in this TA model, so the newly free port is unambiguous.
        set_param(joint,sensorOptions{category},'on');
        ports = get_param(joint,'PortHandles');
        freePorts = ports.RConn(arrayfun(@(h) get_param(h,'Line') == -1,ports.RConn));
        assert(numel(freePorts)==1, ...
            'Expected one new sensor port on joint %d. Inspect the copied model.',j);
        converterName = [model '/' prefixes{category} num2str(j)];
        add_block([model '/q1'],converterName, ...
            'Position',[230+j*190 350+category*180 270+j*190 375+category*180]);
        set_param(converterName,'Unit',units{category});
        converterPorts = get_param(converterName,'PortHandles');
        assert(numel(converterPorts.LConn)==1);
        add_line(model,freePorts,converterPorts.LConn(1),'autorouting','on');
        add_line(model,[prefixes{category} num2str(j) '/1'], ...
            [prefixes{category} 'Mux/' num2str(j)],'autorouting','on');
    end
    assert(isequal(captureStopSettings(joint),stopSettings{j}), ...
        'The limit stiffness/damping settings were unexpectedly changed.');
end
save_system(model,modelFile);
open_system(model);
writematrix(poses,fullfile(outputFolder,'initial_poses_columns_rad.csv'));
save(fullfile(outputFolder,'experiment_settings.mat'), ...
    'poses','limits','stopTime','stopSettings','modelFile');
fprintf('Prepared model: %s\n',modelFile);
fprintf('Output folder: %s\n',outputFolder);
fprintf('Internal damping units kg*m^2/(rad*s) equal N*m*s/rad.\n');
summary = table(); jointResults = table();
if ~runSimulations
    fprintf('Build only: inspect the model, then call run_task7 to run all six cases.\n');
    return;
end

for damping = [0 1]
    for poseNumber = 1:3
        qInitial = poses(:,poseNumber);
        for j = 1:7
            joint = [model '/panda_joint' num2str(j)];
            set_param(joint,'PositionTargetValue',num2str(qInitial(j),17), ...
                'DampingCoefficient',num2str(damping));
            assert(isequal(captureStopSettings(joint),stopSettings{j}));
        end
        name = sprintf('pose%d_B%d',poseNumber,damping);
        caseFolder = fullfile(outputFolder,name); mkdir(caseFolder);
        fprintf('\nRunning %s for %.1f simulated seconds ...\n',name,stopTime);
        try
            out = sim(model,'ReturnWorkspaceOutputs','on');
            % Preserve the raw output before analysis so an analysis error does
            % not discard a completed simulation.
            save(fullfile(caseFolder,'simulation.mat'), ...
                'out','qInitial','damping','poseNumber','limits','-v7.3');
            [runRow,jointTable,data] = measureRun(out,poseNumber,damping,qInitial);
            writetable(runRow,fullfile(caseFolder,'summary.csv'));
            writetable(jointTable,fullfile(caseFolder,'joint_results.csv'));
            columns = arrayfun(@(j) sprintf('q%d_rad',j),1:7,'UniformOutput',false);
            columns = [{'Time_s'},columns, ...
                arrayfun(@(j) sprintf('w%d_rad_s',j),1:7,'UniformOutput',false), ...
                arrayfun(@(j) sprintf('lower%d_Nm',j),1:7,'UniformOutput',false), ...
                arrayfun(@(j) sprintf('upper%d_Nm',j),1:7,'UniformOutput',false)];
            trajectory = array2table([data.t data.Q data.W data.TL data.TU], ...
                'VariableNames',columns);
            writetable(trajectory,fullfile(caseFolder,'trajectory.csv'));
            f = figure('Visible','off');
            subplot(2,1,1); plot(data.t,data.Q); grid on;
            ylabel('Joint position (rad)'); title(strrep(name,'_',' '));
            legend('q1','q2','q3','q4','q5','q6','q7','Location','eastoutside');
            subplot(2,1,2); plot(data.t,data.W); grid on;
            xlabel('Time (s)'); ylabel('Joint velocity (rad/s)');
            savefig(f,fullfile(caseFolder,'joint_motion.fig'));
            print(f,fullfile(caseFolder,'joint_motion.png'),'-dpng','-r150'); close(f);
            if isempty(summary), summary = runRow; else, summary = [summary;runRow]; end %#ok<AGROW>
            if isempty(jointResults), jointResults = jointTable;
            else, jointResults = [jointResults;jointTable]; end %#ok<AGROW>
            writetable(summary,fullfile(outputFolder,'task7_summary.csv'));
            writetable(jointResults,fullfile(outputFolder,'task7_joint_results.csv'));
            disp(runRow);
        catch problem
            fid = fopen(fullfile(caseFolder,'error.txt'),'w');
            if fid ~= -1
                fprintf(fid,'%s',getReport(problem,'extended','hyperlinks','off')); fclose(fid);
            end
            warning('Task7:CaseFailed','%s failed: %s. Details saved in %s', ...
                name,problem.message,caseFolder);
        end
    end
end
save_system(model,modelFile);
fprintf('\nFinished attempted cases. Results: %s\n',outputFolder);
fprintf('Inspect error.txt for any case absent from the summary.\n');
fprintf('A false Settled value means extend the duration and examine the traces.\n');
fprintf('The last successful run can be inspected in Mechanics/Multibody Explorer.\n');
fprintf('Describe the actual final posture; no final-posture description is invented here.\n');
end

function settings = captureStopSettings(joint)
names = {'LowerLimitSpecify','LowerLimitBound','LowerLimitBoundUnits', ...
    'LowerLimitStiffness','LowerLimitStiffnessUnits','LowerLimitDamping', ...
    'LowerLimitDampingUnits','LowerLimitTransitionRegionWidth', ...
    'LowerLimitTransitionRegionWidthUnits','UpperLimitSpecify', ...
    'UpperLimitBound','UpperLimitBoundUnits','UpperLimitStiffness', ...
    'UpperLimitStiffnessUnits','UpperLimitDamping','UpperLimitDampingUnits', ...
    'UpperLimitTransitionRegionWidth','UpperLimitTransitionRegionWidthUnits'};
settings = cellfun(@(name) get_param(joint,name),names,'UniformOutput',false);
end

function [runRow,jointTable,data] = measureRun(out,poseNumber,damping,qInitial)
data.t = out.qArm.Time(:); n = numel(data.t);
data.Q = rowsOfSeven(out.qArm.Data,n);
logs = {'wArm','lowerLimitTorque','upperLimitTorque'};
targets = {'W','TL','TU'};
for k = 1:3
    signal = out.get(logs{k});
    assert(numel(signal.Time)==n && max(abs(signal.Time(:)-data.t))<1e-9, ...
        'Log time vectors must agree.');
    data.(targets{k}) = rowsOfSeven(signal.Data,n);
end
assert(all(isfinite([data.Q(:);data.W(:);data.TL(:);data.TU(:)])), ...
    'The simulation produced nonfinite measurements.');
initialError = max(abs(data.Q(1,:).'-qInitial));
assert(initialError<1e-4, ...
    'Assembled start differs from the requested pose by %.3g rad.',initialError);
torqueThreshold = 1e-4;
lowerHit = nan(7,1); upperHit = nan(7,1);
for j = 1:7
    k = find(abs(data.TL(:,j))>torqueThreshold,1);
    if ~isempty(k), lowerHit(j) = data.t(k); end
    k = find(abs(data.TU(:,j))>torqueThreshold,1);
    if ~isempty(k), upperHit(j) = data.t(k); end
end
assert(data.t(end)-data.t(1)>=1,'Need at least a one-second settling window.');
lastSecond = data.t >= data.t(end)-1;
lastSpeeds = abs(data.W(lastSecond,:));
peakFinalSpeed = max(lastSpeeds(:)); settled = peakFinalSpeed < 0.01;
runRow = table(poseNumber,damping,data.t(end),peakFinalSpeed,settled,initialError, ...
    'VariableNames',{'Pose','B_Nm_s_rad','Duration_s','PeakFinalSecondSpeed_rad_s', ...
    'Settled','InitialAssemblyError_rad'});
jointTable = table(repmat(poseNumber,7,1),repmat(damping,7,1),(1:7).', ...
    lowerHit,upperHit,data.Q(end,:).',data.W(end,:).', ...
    abs(data.TL(end,:).')>torqueThreshold,abs(data.TU(end,:).')>torqueThreshold, ...
    'VariableNames',{'Pose','B_Nm_s_rad','Joint','FirstLowerHit_s', ...
    'FirstUpperHit_s','FinalAngle_rad','FinalSpeed_rad_s','LowerStopActive', ...
    'UpperStopActive'});
end

function A = rowsOfSeven(data,n)
A = squeeze(data);
if isequal(size(A),[7 n]), A = A.'; end
assert(isequal(size(A),[n 7]),'Expected seven synchronized joint signals.');
end
