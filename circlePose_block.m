function [T,p] = circlePose_block(t,Tcenter,radius,period)
% Paste this function into the Simulink MATLAB Function block.
% Inputs: t scalar, Tcenter 4x4, radius scalar, period scalar.
% Outputs: T 4x4 and p 3x1. All values are double.
% The hand follows an xy circle while keeping its orientation unchanged.
angle = 2*pi*t/period;
T = Tcenter;
p = Tcenter(1:3,4)+[radius*cos(angle);radius*sin(angle);0];
T(1:3,4) = p;
end
