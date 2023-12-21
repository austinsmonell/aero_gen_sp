clear
clc
close all

%% Get State Space Inner
K_inner_act = [0, 0];
x0 = [0; 0];
u0 = [0];
[A_inner, B_inner, C, D] = linmod("actuator_controller_inner", x0, u0);
[num, den] = ss2tf(A_inner, B_inner, C, D);
gv_inner = minreal(tf(num, den));

%% LQR Inner
Q_inner = [1 0; 0 1];
R_inner = 0.0012;

[K_inner, ~, E_inner] = lqr(A_inner, B_inner, Q_inner, R_inner);
K_inner_act = K_inner*0.05


%% Get State Space Outer
K_outer = [0, 0, 0, 0];
x0 = [0; 0; 0; 0];
u0 = [0];
[A_outer, B_outer, C, D] = linmod("actuator_controller_outer", x0, u0);
[num, den] = ss2tf(A_outer, B_outer, C, D);
gv_outer = minreal(tf(num, den));

%% LQR Outer
Q_outer = [1 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
R_outer = 50;

[K_outer, ~, E_outer] = lqr(A_outer, B_outer, Q_outer, R_outer);
K_outer_act = [K_outer(1), K_outer(2)*0.05, K_outer(3), K_outer(4)*0.05]