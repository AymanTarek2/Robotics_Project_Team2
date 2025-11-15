clc; clear; close all;

%% ---------------- Symbolic Differentiation Approach ----------------
% Symbols
syms l1 l2 l3 l4 real
syms q1 q2 q3 q4 real           % joint positions (degrees)
syms q1d q2d q3d q4d real       % joint rates (deg/s)

% DH-based transforms (same structure you used; angles in degrees)
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

% Final transform
T = T1*T2*T3*T4;
p = T(1:3,4);           % position
R = T(1:3,1:3);         % rotation

%% ---------------- SYMBOLIC JACOBIAN ----------------
q = [q1 q2 q3 q4];

% Linear velocity part
Jv = jacobian(p, q);     % m/deg

% Angular velocity part
Jw = sym(zeros(3,4));
for i = 1:4
    Rdot_i = diff(R, q(i));        % dR/d(qi)
    S_i = Rdot_i * R.';            % [w_i]x
    w_i = [ S_i(3,2); S_i(1,3); S_i(2,1) ];   % vex(S)
    Jw(:,i) = w_i;                 
end

% Full Jacobian (symbolic)
J = simplify([Jv; Jw]);

disp('=== Symbolic Jacobian J (per-degree) ===');
disp(J);

% Convert to per-radian (standard)
J_rad = simplify(J * (180/pi));
disp('=== Symbolic Jacobian J (per-radian) ===');
disp(J_rad);

% Optional: pretty or latex form
% pretty(J_rad)
% latex_J = latex(J_rad);

%% ---------------- Map joint rates to twist ----------------
qdot = [q1d; q2d; q3d; q4d];     % deg/s
twist = J * qdot;                % [vx;vy;vz; wx;wy;wz]

% Create function handles for numeric evaluation
vars = [l1 l2 l3 l4 q1 q2 q3 q4 q1d q2d q3d q4d];
twist_fun = matlabFunction(simplify(twist), 'Vars', {vars});
J_fun     = matlabFunction(simplify(J),     'Vars', { [l1 l2 l3 l4 q1 q2 q3 q4] });

%% ---------------- Numeric evaluation (your values) ----------------
l1v = 0.0885; l2v = 0.140; l3v = 0.1329; l4v = 0.1105;
q1v = -101.3034; q2v = 91.5482; q3v = 89.0123; q4v =96.5695;        % degrees

% Choose joint rates (deg/s)
q1dv = -4.477675; q2dv = 2.306787; q3dv = 2.771328; q4dv = 4.811676;

tw = twist_fun([l1v l2v l3v l4v q1v q2v q3v q4v q1dv q2dv q3dv q4dv]);
Jnum = J_fun([l1v l2v l3v l4v q1v q2v q3v q4v]);

vx = tw(1); vy = tw(2); vz = tw(3);
wx = tw(4); wy = tw(5); wz = tw(6);

%% ---------------- FK and Euler Angles ----------------
Tn = double(subs(T, [l1 l2 l3 l4 q1 q2 q3 q4], [l1v l2v l3v l4v q1v q2v q3v q4v]));
xn = Tn(1,4); yn = Tn(2,4); zn = Tn(3,4);
Rn = Tn(1:3,1:3);
sy_n = sqrt(Rn(3,2)^2 + Rn(3,3)^2);
if sy_n > 1e-9
    thetay = atan2d(-Rn(3,1), sy_n);
    thetax = atan2d( Rn(3,2), Rn(3,3) );
    thetaz = atan2d( Rn(2,1), Rn(1,1) );
else
    thetay = atan2d(-Rn(3,1), 0);
    thetax = 0;
    thetaz = atan2d(-Rn(1,2), Rn(2,2));
end

%% ---------------- Display ----------------
disp('=== Forward Kinematics (numeric) ===');
fprintf('x = %.4f m\n', xn);
fprintf('y = %.4f m\n', yn);
fprintf('z = %.4f m\n', zn);

disp('Euler ZYX (deg):');
fprintf('roll  (theta_x) = %.4f deg\n', thetax);
fprintf('pitch (theta_y) = %.4f deg\n', thetay);
fprintf('yaw   (theta_z) = %.4f deg\n', thetaz);

disp('=== Jacobian from differentiation (numeric) ===');
disp(Jnum);

disp('=== End-Effector Spatial Velocity (base frame) ===');
fprintf('v = [%.6f  %.6f  %.6f] m/s\n', vx, vy, vz);
fprintf('w = [%.6f  %.6f  %.6f] rad/s\n', wx, wy, wz);
