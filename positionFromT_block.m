function p = positionFromT_block(T)
% Paste into a second Simulink MATLAB Function block after Get Transform.
p = T(1:3,4);
end
