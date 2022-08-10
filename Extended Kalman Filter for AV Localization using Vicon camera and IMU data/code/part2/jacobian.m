clc; clear;

syms p1 p2 p3 q1 q2 q3 p_dot1 p_dot2 p_dot3 bg1 bg2 bg3 ba1 ba2 ba3 w_m a_m 
syms na1 na2 na3 ng1 ng2 ng3 nbg1 nbg2 nbg3 nba1 nba2 nba3 

p = [p1; p2; p3];
q = [q1; q2; q3];
p_dot = [p_dot1; p_dot2; p_dot3];
bg = [bg1; bg2; bg3];
ba = [ba1; ba2; ba3];
X = [p; q; p_dot; bg; ba];
ng = [ng1; ng2; ng3];
na = [na1; na2; na3];
nbg = [nbg1; nbg2; nbg3];
nba = [nba1; nba2; nba3];

g = [0; 0; -9.81];

G = [0 -sin(q3) cos(q2)*cos(q3); 
    0   cos(q3) cos(q2)*sin(q3); 
    1       0           -sin(q2)];

Rx = [1 0 0; 0 cos(q1) -sin(q1); 0 sin(q1) cos(q1)];
Ry = [cos(q2) 0 sin(q2); 0 1 0; -sin(q2) 0 cos(q2)];
Rz = [cos(q3) -sin(q3) 0; sin(q3) cos(q3) 0; 0 0 1];

R = Rz*Ry*Rx;

f_xun = [p_dot; inv(G)*(w_m - bg - ng); g + R*(a_m - ba - na); nbg; nba];


A_t = jacobian(f_xun, [p1 p2 p3 q1 q2 q3 p_dot1 p_dot2 p_dot3 bg1 bg2 bg3 ba1 ba2 ba3])

U_t = jacobian(f_xun, [bg1 bg2 bg3 ba1 ba2 ba3 na1 na2 na3 ng1 ng2 ng3])

