clc; clear; close all;

%% ---------------- Desired End-Effector Position ----------------
xd = 0.05;     % meters
yd = 0.25;     % meters
zd = 0.20;     % meters
pos_des = [xd; yd; zd];

%% ---------------- Robot Parameters ----------------
l1 = 0.0885; 
l2 = 0.140; 
l3 = 0.1329; 
l4 = 0.1105;

%% ---------------- Initial Guess (radians) ----------------
q = deg2rad([30; 60; 60; 90]);   % initial guess
max_iter = 200;              % maximum iterations
tol = 1e-4;                  % position error tolerance (m)
alpha = 0.6;                 % step size (0 < alpha ≤ 1)
max_step = deg2rad(5);       % limit per update (radians)

%% ---------------- Iterative Numerical IK ----------------
for k = 1:max_iter
    % ---- Forward Kinematics ----
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);

    T1 = [ cos(q1), -sin(q1)*cos(pi/2),  sin(q1)*sin(pi/2),  0;
           sin(q1),  cos(q1)*cos(pi/2), -cos(q1)*sin(pi/2),  0;
                0,        sin(pi/2),          cos(pi/2),       l1;
                0,            0,                 0,          1];

    T2 = [ cos(q2+pi/2), -sin(q2+pi/2)*cos(pi),  sin(q2+pi/2)*sin(pi),  l2*cos(q2+pi/2);
           sin(q2+pi/2),  cos(q2+pi/2)*cos(pi), -cos(q2+pi/2)*sin(pi),  l2*sin(q2+pi/2);
                0,             sin(pi),             cos(pi),               0;
                0,                 0,                 0,                   1];

    T3 = [ cos(q3), -sin(q3)*cos(pi),  sin(q3)*sin(pi),  l3*cos(q3);
           sin(q3),  cos(q3)*cos(pi), -cos(q3)*sin(pi),  l3*sin(q3);
                0,        sin(pi),         cos(pi),          0;
                0,            0,               0,            1];

    T4 = [ cos(q4), -sin(q4)*cos(0),  sin(q4)*sin(0),  l4*cos(q4);
           sin(q4),  cos(q4)*cos(0), -cos(q4)*sin(0),  l4*sin(q4);
                0,         sin(0),          cos(0),          0;
                0,            0,               0,            1];

    T = T1*T2*T3*T4;
    p = T(1:3,4);     % current end-effector position

    % ---- Position Error ----
    e = pos_des - p;           
    err_norm = norm(e);

    % ---- Numerical Jacobian ----
    dq = 1e-4;  % small radian perturbation
    Jv = zeros(3,4);
    for i = 1:4
        q_pert = q;
        q_pert(i) = q_pert(i) + dq;
        % Forward kinematics for perturbed joint
        q1p=q_pert(1); q2p=q_pert(2); q3p=q_pert(3); q4p=q_pert(4);

        T1p=[ cos(q1p), -sin(q1p)*cos(pi/2), sin(q1p)*sin(pi/2), 0;
              sin(q1p),  cos(q1p)*cos(pi/2), -cos(q1p)*sin(pi/2), 0;
                   0,    sin(pi/2), cos(pi/2), l1;
                   0, 0, 0, 1];
        T2p=[ cos(q2p+pi/2), -sin(q2p+pi/2)*cos(pi), sin(q2p+pi/2)*sin(pi), l2*cos(q2p+pi/2);
              sin(q2p+pi/2),  cos(q2p+pi/2)*cos(pi), -cos(q2p+pi/2)*sin(pi), l2*sin(q2p+pi/2);
                   0,          sin(pi), cos(pi), 0;
                   0, 0, 0, 1];
        T3p=[ cos(q3p), -sin(q3p)*cos(pi), sin(q3p)*sin(pi), l3*cos(q3p);
              sin(q3p),  cos(q3p)*cos(pi), -cos(q3p)*sin(pi), l3*sin(q3p);
                   0, sin(pi), cos(pi), 0;
                   0, 0, 0, 1];
        T4p=[ cos(q4p), -sin(q4p)*cos(0), sin(q4p)*sin(0), l4*cos(q4p);
              sin(q4p),  cos(q4p)*cos(0), -cos(q4p)*sin(0), l4*sin(q4p);
                   0, sin(0), cos(0), 0;
                   0, 0, 0, 1];
        Tp = T1p*T2p*T3p*T4p;
        pp = Tp(1:3,4);
        Jv(:,i) = (pp - p) / dq;  % m/rad
    end

    % ---- Compute update ----
    dq_update = alpha * pinv(Jv) * e;              % scaled step
    dq_update = max(min(dq_update, max_step), -max_step);  % limit movement per joint

    % ---- Update joint angles ----
    q = q + dq_update;
    q = wrapToPi(q);   % keep angles within [-pi, pi]

    % ---- Display iteration progress ----
    fprintf('Iter %3d | Error = %.6f m\n', k, err_norm);

    % ---- Convergence check ----
    if err_norm < tol
        fprintf('Converged after %d iterations.\n', k);
        break;
    end
end

%% ---------------- Display Results ----------------
fprintf('\nFinal Joint Angles (deg):\n');
fprintf('q1 = %.4f°\nq2 = %.4f°\nq3 = %.4f°\nq4 = %.4f°\n', rad2deg(q(1)), rad2deg(q(2)), rad2deg(q(3)), rad2deg(q(4)));
fprintf('\nFinal End-Effector Position:\n');
fprintf('x = %.4f, y = %.4f, z = %.4f\n', p(1), p(2), p(3));
fprintf('Final Error Norm = %.6f m\n', err_norm);
