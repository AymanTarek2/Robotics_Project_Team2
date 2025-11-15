clc; clear; close all;
%% -------- Inverse Velocity using desired [vx, vy, vz] only --------
% Symbols
syms l1 l2 l3 l4 real
syms q1 q2 q3 q4 real           % joint angles (deg)

% DH transforms (same as your FK)
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

% Forward kinematics and position
T = T1*T2*T3*T4;
p = T(1:3,4);

% Linear velocity Jacobian (m/deg)
q = [q1 q2 q3 q4];
Jv = jacobian(p, q);

% Numeric function for Jv
Jv_fun = matlabFunction(simplify(Jv), 'Vars', { [l1 l2 l3 l4 q1 q2 q3 q4] });

%% ---------------- Numeric evaluation & inverse velocity ----------------
% Robot parameters & configuration (deg)
l1v = 0.0885; l2v = 0.140; l3v = 0.1329; l4v = 0.1105;
qv   = [-101.3034 91.5482 89.0123 96.5695];

% Evaluate Jv at current q
Jv_num = Jv_fun([l1v l2v l3v l4v qv]);

% === Set your desired linear velocity in BASE frame (m/s) ===
vx_des = 0.019036;  vy_des = -0.006411;  vz_des = -0.013865;
v_des = [vx_des; vy_des; vz_des];

% Inverse velocity (minimum-norm) solution: qdot (deg/s)
qdot_deg = pinv(Jv_num) * v_des;

% Quick check (reconstruct v)
v_rec = Jv_num * qdot_deg;

%% ---------------- Display ----------------
disp('=== Inverse velocity result (using only Jv) ===');
fprintf('Desired v = [%.6f  %.6f  %.6f] m/s\n', v_des);
fprintf('qdot (deg/s) = [%.6f  %.6f  %.6f  %.6f]\n', qdot_deg);
fprintf('Reconstructed v = [%.6f  %.6f  %.6f] m/s\n', v_rec);
disp(pinv(Jv_num))
