clc
clear
close all

%% Load Data
addpath('../aero_gen_library_sp/')

load("state1.mat");
load("wing1.mat");

%% Creat Target Flight Path

% [flight_path_1, ref1] = ellipse_flight_path(138, 58, 58, deg2rad(10.8), deg2rad(-3), deg2rad(0), deg2rad(0), 200, 1);

%% System Parameters
viz_on = 1;
ac1_forces = 1;
ac1_moments = 1;

gravity = [0 , 0, 9.81];
rho = 1.225;
k_line = 100;
d =150;
WindVelocity = [12, 0, 0];%[12, 0, 0]


%% Aircraft Parameters
roll_moment_1 = 0;
pitch_moment_1 = 0;
yaw_moment_1 = 0.06;%+
cl_1 = 6.2;

roll_c_1 = deg2rad(12.6);%-5
pitch_c = deg2rad(-10);%-18
k_roll = 10;%100
k_pitch = 1000;%1000

%% Constants Vector
coeff = cell2mat(wing1.Coefficients.Values);
coeff_ctr = cell2mat(wing1.Surfaces.Coefficients.Values);

param = [state1.Mass, rho, wing1.ReferenceArea, wing1.ReferenceLength, wing1.ReferenceSpan, gravity, WindVelocity,...
         state1.Inertia{1, 1}, state1.Inertia{2, 2}, state1.Inertia{3, 3}, k_roll, k_pitch, d, k_line,...
         coeff(1, 1), coeff(1, 3), coeff(2, 6), coeff(3, 1), coeff(3, 3), coeff(4, 8), coeff(4,8), coeff(5, 3), coeff(5, 5), coeff(6, 6), coeff(6, 9),...
         coeff_ctr(4, 1), coeff_ctr(5, 2), coeff_ctr(6, 3), yaw_moment_1, pitch_c, roll_c_1];

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

%% Load Initial Conditions

load("Flight Configurations\IC_Saves\150m_SS_1.mat");

InitPosition1 = IC_ss(1:3)';
InitVelocity1 = IC_ss(4:6)';
InitAcc1 = IC_ss(7:9)';
InitEuler1 = IC_ss(10:12)';
InitPQR1 = IC_ss(13:15)';
InitPQRDot1 = IC_ss(16:18)';


%% Save IC
% name = 'IC_test';
% save_final_state(name, out);

%% Set Parameters
[state1] = set_parameters(WindVelocity,InitPosition1, InitVelocity1, InitEuler1, InitPQR1, state1);

%% Save Config
name = 'test';
% flight_config = save_configuration(name,viz_on, ac1_forces, ac2_forces, ac1_moments, ac2_moments, WindVelocity, k_line, gravity, d, rho, ...
%                                    pitch_c, k_pitch, pitch_moment_1, pitch_moment_2, roll_c_1, roll_c_2,k_roll, roll_moment_1, roll_moment_2, yaw_moment_1, yaw_moment_2, ...
%                                    InitEuler1, InitEuler2, InitPosition1, InitPosition2, InitPQR1, InitPQR2, InitVelocity1, InitVelocity2, InitAcc1, InitAcc2, InitTheta, InitThetaDot, ...
%                                    k_damp, c, moi, r, theta_0);

%save 'Flight Configurations/test.mat' flight_config;

%% Create IC
ac1_state = [InitPosition1, InitVelocity1, InitEuler1, InitPQR1];
ctr1 = [0, 0, 0]; thr = [0]; pod_ctr = [0, 0];

%% Generate Jacobian Function
addpath('../Adigator');
% need to run startupadigator to create new derivitive functions
% startupadigator;
% 
% x = adigatorCreateDerivInput([12, 1], 'x');
% u = adigatorCreateAuxInput([4 1]);
% adigator('get_full_state', {x,u, param'}, 'deriv2');
% 
% x = adigatorCreateDerivInput([4, 1], 'x');
% u = adigatorCreateAuxInput([12 1]);
% adigator('get_full_state', {u, x, param'}, 'deriv_ctr');

%% Create MPC Object
tic;
nlobj = generate_multi_mpc(ac1_state, [ctr1,  thr]', param);

%% Run Sim
stop_time = 10;
time_step = 0.001;

out = sim('aero_gen.slx');
toc

%% Plot Output
time = out.tout;
y = out.state.signals.values;

%3d plot flightpath
figure(1);
hold on;
plot3(y(:, 1), y(:, 2), y(:, 3));
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Flight Path [1 Cycle]')

%radius
figure(4);
plot(time, sqrt(y(:, 1).^2 + y(:, 2).^2 + y(:, 3).^2));
hold on;
xlabel('Time [s]');
ylabel('Radius [m]');
title('Radius vs Time');

%% Test Get State Function
% ac1_state = out.state.signals.values(1, 1:18);
coeff = cell2mat(wing1.Coefficients.Values);
coeff_ctr = cell2mat(wing1.Surfaces.Coefficients.Values);
tic;
[t, y] = run_sim_states(stop_time, ac1_state, [ctr1, thr]', param);
toc

%Plot from sim states
%3d plot
figure(1);
hold on;
plot3(y(:, 1), y(:, 2), y(:, 3));
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Flight Path [1 Cycle]')
%radius
figure(4);
plot(t, sqrt(y(:, 1).^2 + y(:, 2).^2 + y(:, 3).^2));
xlabel('Time [s]');
ylabel('Radius [m]');
title('Radius vs Time');