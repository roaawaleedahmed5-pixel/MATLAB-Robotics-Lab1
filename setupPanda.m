function [panda,qReady,ee] = setupPanda()
panda = loadrobot('frankaEmikaPanda','DataFormat','column','Gravity',[0 0 -9.81]);
% I remove the finger branches so that all configuration entries are arm joints.
% The hand stays in the tree and remains the end effector for the lab.
fingerNames = {'panda_leftfinger','panda_rightfinger'};
for k = 1:numel(fingerNames)
    if ismember(fingerNames{k},panda.BodyNames)
        removeBody(panda,fingerNames{k});
    end
end
ee = 'panda_hand';
assert(ismember(ee,panda.BodyNames));
assert(numel(homeConfiguration(panda))==7, ...
    'This model has extra moving joints. Inspect showdetails before proceeding.');
% Explicit ready-pose assumption; use a lecturer-supplied pose if provided.
qReady = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4];
limits = armJointLimits(panda);
assert(all(qReady >= limits(:,1) & qReady <= limits(:,2)));
end
