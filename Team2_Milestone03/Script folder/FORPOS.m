clc; clear; close all;

% Assign numerical values to link lengths
l1 = 0.0885; l2 = 0.140; l3 = 0.1329; l4 = 0.1105;

% Given Joint Values (in degrees)
q1 = 78.6881; q2 = -44.1315; q3 = 85.4983; q4 = 99.7213;

% Transformation Matrices using DH Parameters
T1 = [ cosd(q1), -sind(q1)*cosd(90),  sind(q1)*sind(90),  0;
       sind(q1),  cosd(q1)*cosd(90), -cosd(q1)*sind(90),  0;
            0,        sind(90),          cosd(90),       l1;
            0,            0,                 0,          1];

T2 = [ cosd(q2+90), -sind(q2+90)*cosd(180),  sind(q2+90)*sind(180),  l2*cosd(q2+90);
       sind(q2+90),  cosd(q2+90)*cosd(180), -cosd(q2+90)*sind(180),  l2*sind(q2+90);
            0,             sind(180),             cosd(180),               0;
            0,                 0,                     0,                   1];

T3 = [ cosd(q3), -sind(q3)*cosd(180),  sind(q3)*sind(180),  l3*cosd(q3);
       sind(q3),  cosd(q3)*cosd(180), -cosd(q3)*sind(180),  l3*sind(q3);
            0,        sind(180),            cosd(180),            0;
            0,            0,                   0,                 1];

T4 = [ cosd(q4), -sind(q4)*cosd(0),  sind(q4)*sind(0),  l4*cosd(q4);
       sind(q4),  cosd(q4)*cosd(0), -cosd(q4)*sind(0),  l4*sind(q4);
            0,         sind(0),          cosd(0),            0;
            0,            0,               0,               1];

% Compute Final Transformation Matrix
T_final = T1 * T2 * T3 * T4;

% Extract End-Effector Position
x = T_final(1,4);
y = T_final(2,4);
z = T_final(3,4);

% -------- Manual Euler angles extraction (ZYX: yaw-pitch-roll) --------
% R = Rz(yaw)*Ry(pitch)*Rx(roll)
R = T_final(1:3,1:3);

% Helper quantity to detect gimbal lock (when sy ~ 0)
sy = sqrt(R(3,2)^2 + R(3,3)^2);   % = sqrt(R32^2 + R33^2)

if sy > 1e-9
    % Regular case
    thetay = atan2d(-R(3,1), sy);       % pitch about Y
    thetax = atan2d( R(3,2), R(3,3) );  % roll about X
    thetaz = atan2d( R(2,1), R(1,1) );  % yaw  about Z
else
    % Gimbal lock: pitch is ±90°, set roll = 0 and absorb it into yaw
    thetay = atan2d(-R(3,1), 0);        % ±90 deg
    thetax = 0;                         
    thetaz = atan2d(-R(1,2), R(2,2));   % yaw adjusted
end
% ----------------------------------------------------------------------

% Display the Results
disp('End-Effector Position:');
fprintf('x = %.4f m\n', x);
fprintf('y = %.4f m\n', y);
fprintf('z = %.4f m\n', z);

disp('End-Effector Orientation (Euler Angles - ZYX sequence):');
fprintf('roll  (theta_x) = %.4f deg\n', thetax);
fprintf('pitch (theta_y) = %.4f deg\n', thetay);
fprintf('yaw   (theta_z) = %.4f deg\n', thetaz);
