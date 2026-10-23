%% Run after simulation, with Single simulation output named out.
% To Workspace blocks must be Timeseries named pCmd, pAch, ikStatus and qLog.
t = out.pCmd.Time;
Pcmd = samplesByRow(out.pCmd.Data,numel(t),3);
Pach = samplesByRow(out.pAch.Data,numel(t),3);
status = out.ikStatus.Data(:);
Qsim = samplesByRow(out.qLog.Data,numel(t),7);
assert(numel(status)==numel(t));
assert(max(abs(out.pAch.Time-t)) < 1e-9);
positionError = vecnorm(Pcmd-Pach,2,2);
fprintf('Peak position error: %.6g m (%.6g mm)\n', ...
    max(positionError),1000*max(positionError));
fprintf('Non-success samples: %d out of %d\n',sum(status~=1),numel(status));
fprintf('Maximum sample-to-sample joint changes, rad:\n');
disp(max(abs(diff(Qsim)),[],1));
figure; plot3(Pcmd(:,1),Pcmd(:,2),Pcmd(:,3),'k--','LineWidth',2); hold on;
plot3(Pach(:,1),Pach(:,2),Pach(:,3),'r'); axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); legend('Commanded','Achieved');
figure; plot(t,positionError*1000); grid on;
xlabel('Time (s)'); ylabel('Position error (mm)');
figure; plot(t,Qsim); grid on; xlabel('Time (s)'); ylabel('Joint position (rad)');

function A = samplesByRow(data,n,width)
A = squeeze(data);
if isequal(size(A),[width n]), A = A.'; end
assert(isequal(size(A),[n width]),'Check To Workspace dimensions and Timeseries format.');
end
