clc; clear; close all;

% Define symbolic variables for joint angles and link lengths
syms q1 q2 q3 q4  real
syms l1 l2 l3 l4  real

% Assign numerical values to link lengths
l1 = 0.04355; l2 = 0.140; l3 = 0.1329; l4 = 0.01213; 


% Desired end-effector position
 xd = 1;
    yd = -1;
    zd = 0.4;
epsilon = 0.01;  % Convergence
max_iter = 100;  % Maximum iterations

  q = [30;30;30;30];
    % Forward Kinematics Transformation Matrices 

% Transformation Matrices using DH Parameters
T1 = [ cosd(q1), -sind(q1)*cosd(90),  sind(q1)*sind(90),  0;
       sind(q1),  cosd(q1)*cosd(90),  -cosd(q1)*sind(90),  0;
            0,        sind(90),   cosd(90),      l1;
            0,        0,   0,      1];


T2 = [ cosd(q2+90), -sind(q2+90)*cosd(180),  sind(q2+90)*sind(180),  l2*cosd(q2+90);
       sind(q2+90),  cosd(q2+90)*cosd(180),  -cosd(q2+90)*sind(180),  l2*sind(q2+90);
            0,        sind(180),   cosd(180),      0;
            0,        0,   0,      1];

T3 = [ cosd(q3), -sind(q3)*cosd(180),  sind(q3)*sind(180),  l3*cosd(q3);
       sind(q3),  cosd(q3)*cosd(180),  -cosd(q3)*sind(180), l3*sind(q3);
            0,        sind(180),   cosd(180),      0;
            0,        0,   0,      1];

T4 = [ cosd(q4), -sind(q4)*cosd(0),  sind(q4)*sind(0),  l4*cosd(q4);
       sind(q4),  cosd(q4)*cosd(0),  -cosd(q4)*sind(0),  l4*sind(q4);
            0,        sind(q4),   cosd(0),      l4;
            0,        0,   0,      1];

% Compute Final Transformation Matrix
T_final = T1 * T2 * T3 * T4;

% Extract End-Effector Position
x = T_final(1,4);
y = T_final(2,4);
z = T_final(3,4);

% Compute Jacobian Matrix Symbolically
J = jacobian([x; y; z], [q1, q2, q3, q4]);

% Newton-Raphson Iterative Solution
for iter = 1:max_iter
    % Substitute joint angles and link lengths
    subs_vars = [q1,q2, q3, q4 , l1, l2, l3, l4];
    subs_values = [transpose(q) l1 l2 l3 l4 ];

    % Compute Current End-Effector Position
    x_val = double(subs(x, subs_vars, subs_values));
    y_val = double(subs(y, subs_vars, subs_values));
    z_val = double(subs(z, subs_vars, subs_values));
    
    % Compute Error Vector
    F_val = [x_val - xd; y_val - yd; z_val - zd];


    % Check for Convergence
    if norm(F_val) < epsilon
        fprintf('Converged in %d iterations\n', iter);
       
        break;
    end

    % Evaluate Jacobian for Current Joint Angles
    J_val = double(subs(J, subs_vars, subs_values));

    % Compute Joint Angle Update Using Pseudoinverse
    delta_q = -pinv(J_val) * F_val;

    % Update Joint Angles
    q = q + delta_q;
end
% Print iteration result
if iter == max_iter
    fprintf('Reached maximum iterations (%d) without full convergence.\n', max_iter);
else
    fprintf('Converged in %d iterations\n', iter);
end
qout=q;

% Display Final Joint Angles
disp('Final Joint Angles (radians):');
disp(qout);