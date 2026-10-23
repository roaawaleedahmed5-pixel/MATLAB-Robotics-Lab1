%% Task 1 - Frames and transforms
clear; clc; close all;


% Camera position and orientation relative to the table as stated in the
% lab
cameraPosition = [0.8 -0.3 1.2];
yaw = deg2rad(90);
pitch = deg2rad(-30);
roll = 0;

% Building the camera pose relative to the table
T_table_camera = trvec2tform(cameraPosition) * ...
    eul2tform([yaw pitch roll], 'ZYX');

% Relating the object pose to the table and extracting its position and
% quat angles to be displayed
pCamera = [0; 0; 0.6; 1];
pTable = T_table_camera * pCamera;
quaternion = tform2quat(T_table_camera); % [w x y z]
axisAngle = tform2axang(T_table_camera); % [axis_x axis_y axis_z angle_rad]

% I use the inverse transformation to check that I recover the original point.
T_camera_table = inv(T_table_camera);
pRecovered = T_camera_table * pTable;
recoveryError = norm(pRecovered(1:3) - pCamera(1:3));

disp('Camera pose in table coordinates:'); disp(T_table_camera);
disp('Object position in the table frame, m:'); disp(pTable(1:3));
disp('Quaternion [w x y z]:'); disp(quaternion);
disp('Axis-angle [axis_x axis_y axis_z angle_rad]:'); disp(axisAngle);
disp('Recovered camera-frame coordinates, m:'); disp(pRecovered(1:3));
fprintf('Recovery error: %.3e m\n', recoveryError);
assert(recoveryError < 1e-12);

fprintf('Camera local +z expressed in table axes:\n');
disp(T_table_camera(1:3,3));
