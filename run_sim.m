clc
clear
close all

%% Load Data
addpath('../aero_gen_library_sp/')

load("state1.mat");
load("state2.mat");
load("wing1.mat");

%% Creat Target Flight Path
% ellipse_flight_path(138, 58, 58, deg2rad(-10.8), deg2rad(-3), deg2rad(0), deg2rad(0), 5000, -1);
% ellipse_flight_path(138, 58, 58, deg2rad(10.8), deg2rad(-3), deg2rad(0), deg2rad(0), 5000, 1);

[flight_path_2, ref2] = ellipse_flight_path(138, 58, 58, deg2rad(-10.8), deg2rad(-3), deg2rad(0), deg2rad(0), 200, -1);
[flight_path_1, ref1] = ellipse_flight_path(138, 58, 58, deg2rad(10.8), deg2rad(-3), deg2rad(0), deg2rad(0), 200, 1);

%% System Parameters
viz_on = 1;
ac1_forces = 1;
ac2_forces = 1;
ac1_moments = 1;
ac2_moments = 1;

gravity = [0 , 0, 9.81];
rho = 1.225;
k_line = 30000;%30000%500%110
d =300;
WindVelocity = [12, 0, 0];%[12, 0, 0]


%% Aircraft Parameters
roll_moment_1 = 0;
pitch_moment_1 = 0;
yaw_moment_1 = 0.06;%+
cl_1 = 6.2;

roll_moment_2 = 0;
pitch_moment_2 = 0;
yaw_moment_2 = -0.06;%-
cl_2 = 6.2;

roll_c_1 = deg2rad(12.6);%-5
roll_c_2 = deg2rad(-12.6);%5
pitch_c = deg2rad(-10);%-18
k_roll = 100;%100
k_pitch = 1000;%1000

%% Generator Parameters

moi = 0.01;
r = 0.1;
k_damp = 0;%30
c = 0;%30;
theta_0 = 0;

%% Constants Vector
coeff = cell2mat(wing1.Coefficients.Values);
coeff_ctr = cell2mat(wing1.Surfaces.Coefficients.Values);

param = [state1.Mass, rho, wing1.ReferenceArea, wing1.ReferenceLength, wing1.ReferenceSpan, gravity(1), gravity(2) gravity(3), WindVelocity,...
         state1.Inertia{1, 1}, state1.Inertia{2, 2}, state1.Inertia{3, 3}, k_roll, k_pitch, r, d, k_line, k_damp, c, moi, ...
         coeff(1, 1), coeff(1, 3), coeff(2, 6), coeff(3, 1), coeff(3, 3), coeff(4, 8), coeff(4,8), coeff(5, 3), coeff(5, 5), coeff(6, 6), coeff(6, 9),...
         coeff_ctr(4, 1), coeff_ctr(5, 2), coeff_ctr(6, 3), yaw_moment_1, yaw_moment_2, pitch_c, roll_c_1, roll_c_2];

%% Initial Conditions
% 
% 
% InitPosition1 = [138, 0.01, 0.01];
% InitVelocity1 = [20, 0.01, 0.01];
% InitAcc1 = [0, 0, 0];
% InitEuler1 = [0, 0, 0]*pi/180;
% InitPQR1 = [0, 0, 0];
% InitPQRDot1 = [0, 0, 0];
% 
% InitPosition2 = [138, 10, 0.01];
% InitVelocity2 = [20, 0.01, 0.01];
% InitAcc2 = [0, 0, 0];
% InitEuler2 = [0, 0, 0]*pi/180;
% InitPQR2 = [0, 0, 0];
% InitPQRDot2 = [0, 0, 0];
% 
% InitTheta = 0;
% InitThetaDot = 0;

%% Load Initial Conditions

load("Flight Configurations\IC_Saves\300m_test.mat");

InitPosition1 = IC_ss(1:3)';
InitVelocity1 = IC_ss(4:6)';
InitAcc1 = IC_ss(7:9)';
InitEuler1 = IC_ss(10:12)';
InitPQR1 = IC_ss(13:15)';
InitPQRDot1 = IC_ss(16:18)';


InitPosition2 = IC_ss(19:21)';
InitVelocity2 = IC_ss(22:24)';
InitAcc2 = IC_ss(25:27)';
InitEuler2 = IC_ss(28:30)';
InitPQR2 = IC_ss(31:33)';
InitPQRDot2 = IC_ss(34:36)';

InitTheta = IC_ss(37);
InitThetaDot = IC_ss(38);

%% Save IC
% name = 'IC_test';
% save_final_state(name, out);

%% Set Parameters
[state1,state2] = set_parameters(WindVelocity,InitPosition1, InitVelocity1,...
                                 InitEuler1, InitPQR1, InitPosition2, ...
                                 InitVelocity2,InitEuler2, InitPQR2, ...
                                 state1, state2);

%% Save Config
name = 'test';
flight_config = save_configuration(name,viz_on, ac1_forces, ac2_forces, ac1_moments, ac2_moments, WindVelocity, k_line, gravity, d, rho, ...
                                   pitch_c, k_pitch, pitch_moment_1, pitch_moment_2, roll_c_1, roll_c_2,k_roll, roll_moment_1, roll_moment_2, yaw_moment_1, yaw_moment_2, ...
                                   InitEuler1, InitEuler2, InitPosition1, InitPosition2, InitPQR1, InitPQR2, InitVelocity1, InitVelocity2, InitAcc1, InitAcc2, InitTheta, InitThetaDot, ...
                                   k_damp, c, moi, r, theta_0);

%save 'Flight Configurations/test.mat' flight_config;

%% Create MPC Object
ac1_state = [InitPosition1, InitVelocity1, InitEuler1, InitPQR1];
ac2_state = [InitPosition2, InitVelocity2, InitEuler2, InitPQR2];
gen_state = [InitTheta, InitThetaDot];
ctr1 = [0, 0, 0]; ctr2 = [0, 0, 0]; thr = [0, 0]; pod_ctr = [0, 0, 0, 0];
tic;
nlobj = generate_multi_mpc([ac1_state, ac2_state, gen_state], [ctr1, ctr2, thr, pod_ctr]', param);

%% Run Sim
stop_time = 10;
time_step = 0.001;

%out = sim('aero_gen.slx');
toc

%% Plot Output
time = out.tout;
y = out.state.signals.values;

%3d plot flightpath
figure(1);
hold on;
plot3(y(:, 1), y(:, 2), y(:, 3));
hold on;
plot3(y(:, 19), y(:, 20), y(:, 21));
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Flight Path [1 Cycle]')
legend('wing1', 'wing2');

%plot generator responce
figure(2);
plot(time, y(:, 37));
hold on;
plot(time, y(:, 38));
xlabel('Time [s]');
ylabel('rad & rad/s');
title('Generator Responce');
legend('theta', 'theta dot');

%radius
figure(3);
plot(time, sqrt(y(:, 1).^2 + y(:, 2).^2 + y(:, 3).^2));
hold on;
plot(time, sqrt(y(:, 19).^2 + y(:, 20).^2 + y(:, 21).^2));
xlabel('Time [s]');
ylabel('Radius [m]');
title('Radius vs Time');
legend('R wing1', 'R wing2');

%gamma
figure(4);
plot(time, y(:, 39));
hold on;
plot(time, y(:, 40));
plot(time, y(:, 41));
xlabel('Time [s]');
ylabel('Gamma [rad]');
title('Gamma vs Time');
legend('Gamma 1', 'Gamma 2', 'Delta Gamma');

%% Test Get State Function
% ac1_state = out.state.signals.values(1, 1:18);
% ac2_state = out.state.signals.values(1, 19:36);
% gen_state = out.state.signals.values(1, 37:38);
coeff = cell2mat(wing1.Coefficients.Values);
coeff_ctr = cell2mat(wing1.Surfaces.Coefficients.Values);
tic;
[t, y] = run_sim_states(stop_time, ac1_state, ac2_state, gen_state, ctr1, ctr2, param);
toc



%Plot from sim states
%3d plot
figure(1);
hold on;
plot3(y(:, 1), y(:, 2), y(:, 3));
hold on;
plot3(y(:, 13), y(:, 14), y(:, 15));
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Flight Path [1 Cycle]')
legend('wing1', 'wing2');
%radius
figure(6);
plot(t, sqrt(y(:, 1).^2 + y(:, 2).^2 + y(:, 3).^2));
hold on;
plot(t, sqrt(y(:, 13).^2 + y(:, 14).^2 + y(:, 15).^2));
xlabel('Time [s]');
ylabel('Radius [m]');
title('Radius vs Time');
legend('R wing1', 'R wing2');
%generator responce
figure(7);
plot(t, y(:, 25));
hold on;
plot(t, y(:, 26));
xlabel('Time [s]');
ylabel('rad & rad/s');
title('Generator Responce');
legend('theta', 'theta dot');