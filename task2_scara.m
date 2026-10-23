%% Task 2 - Model and verify the SCARA
clear; clc; close all;
[scara,p] = makeSCARA();
showdetails(scara);
disp('DH columns: a (m), alpha (rad), d (m), theta (rad)'); disp(p.DH);
disp('Limits: angles in rad; third row in m'); disp(p.limits);

configs = [deg2rad(0)    deg2rad(40)   deg2rad(-60);
           deg2rad(60)   deg2rad(-80)  deg2rad(100);
           0.02         0.10         0.18;
           deg2rad(-30) deg2rad(30)   deg2rad(-20)];

figure;
for k = 1:3
    q = configs(:,k);
    assert(all(q >= p.limits(:,1) & q <= p.limits(:,2)));
    subplot(1,3,k);
    show(scara,q,'Frames','on','Visuals','off');
    title(sprintf('Configuration %d',k)); view(35,25); axis equal;
    % This model has frames and skeletons rather than detailed CAD geometry.
    Tmanual = dhStd(p.L1,0,p.h,q(1)) * ...
              dhStd(p.L2,0,0,q(2)) * ...
              dhStd(0,0,q(3),0) * dhStd(0,0,0,q(4));
    Tmatlab = getTransform(scara,q,'link4');
    bodyError = norm(Tmanual-Tmatlab,'fro');
    TtoolManual = Tmanual*trvec2tform([0 0 -p.toolDrop]);
    toolError = norm(TtoolManual-getTransform(scara,q,'tool'),'fro');
    fprintf('Config %d: body error %.3e; tool error %.3e\n',k,bodyError,toolError);
    assert(bodyError < 1e-10 && toolError < 1e-10);
end
