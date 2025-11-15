clc; clear; close all;

% Given Joint Values (in degress)
syms q1 q2 q3 q4 l1 l2 l3 l4
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

% Display the Results Symbolically
%disp('End-Effector Position (Symbolic):');
fprintf('x = %s\n', char(x));
fprintf('y = %s\n', char(y));
fprintf('z = %s\n', char(z));
