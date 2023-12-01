clear
clc
close all

%% Get State Space
K = [1, 0, 0, 0, 0, 0];
x0 = [0; 0; 0; 0; 0; 0];
u0 = [0];
[A, B, C, D] = linmod("actuator_controller", x0, u0);
[num, den] = ss2tf(A, B, C, D);
gv = minreal(tf(num, den));

%% LQR
Q = [1 0 0 0 0 0; 0 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 10];
R = 5;

[K, S, E] = lqr(A, B, Q, R)