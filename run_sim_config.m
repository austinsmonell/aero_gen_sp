clc
clear
close all

%% Load Data
addpath('../aero_gen_library/');

load("state1.mat");
load("state2.mat");
load("wing1.mat");
load('Flight Configurations\test.mat')

%% System Parameters
viz_on = flight_config.sys_params.viz_on;
ac1_forces = flight_config.sys_params.ac1_forces;
ac2_forces = flight_config.sys_params.ac2_forces;
ac1_moments = flight_config.sys_params.ac1_moments;
ac2_moments = flight_config.sys_params.ac2_moments;

gravity = flight_config.sys_params.gravity;
k_line = flight_config.sys_params.k_line;
d = flight_config.sys_params.d;
WindVelocity = flight_config.sys_params.wind;
rho = flight_config.sys_params.rho;

%% Aircraft Parameters
roll_moment_1 = flight_config.aircraft_params.roll_moment_coeff_1;
pitch_moment_1 = flight_config.aircraft_params.pitch_moment_coeff_1;
yaw_moment_1 = flight_config.aircraft_params.yaw_moment_coeff_1;

roll_moment_2 = flight_config.aircraft_params.roll_moment_coeff_2;
pitch_moment_2 = flight_config.aircraft_params.pitch_moment_coeff_2;
yaw_moment_2 = flight_config.aircraft_params.yaw_moment_coeff_2;

roll_c_1 = flight_config.aircraft_params.roll_control_input_1;
roll_c_2 = flight_config.aircraft_params.roll_control_input_2;
pitch_c = flight_config.aircraft_params.pitch_control_input;
k_roll = flight_config.aircraft_params.roll_k;
k_pitch = flight_config.aircraft_params.pitch_k;

%% Initial Conditions
InitPosition1 = flight_config.ic.position_1;
InitVelocity1 = flight_config.ic.velocity_1;
InitEuler1 = flight_config.ic.euler_1;
InitPQR1 = flight_config.ic.pqr_1;
InitAcc1 = flight_config.ic.acc_1;

InitPosition2 = flight_config.ic.position_2;
InitVelocity2 = flight_config.ic.velocity_2;
InitEuler2 = flight_config.ic.euler_2;
InitPQR2 = flight_config.ic.pqr_2;
InitAcc2 = flight_config.ic.acc_2;

InitTheta = flight_config.ic.theta_init;
InitThetaDot = flight_config.ic.theta_dot_init;



%% Generator Parameters

theta_0 = flight_config.gen_params.theta_0;
moi = flight_config.gen_params.moi;
r = flight_config.gen_params.r;
c = flight_config.gen_params.c;



%% Set Parameters
state1.Environment.WindVelocity = WindVelocity;
state1.XN = InitPosition1(1);
state1.XE = InitPosition1(2);
state1.XD = InitPosition1(3);
state1.U = InitVelocity1(1);
state1.V = InitVelocity1(2);
state1.W = InitVelocity1(3);
state1.Phi = InitEuler1(1);
state1.Theta = InitEuler1(2);
state1.Psi = InitEuler1(3);
state1.P = InitPQR1(1);
state1.Q = InitPQR1(2);
state1.R = InitPQR1(3);

state2.Environment.WindVelocity = WindVelocity;
state2.XN = InitPosition2(1);
state2.XE = InitPosition2(2);
state2.XD = InitPosition2(3);
state2.U = InitVelocity2(1);
state2.V = InitVelocity2(2);
state2.W = InitVelocity2(3);
state2.Phi = InitEuler2(1);
state2.Theta = InitEuler2(2);
state2.Psi = InitEuler2(3);
state2.P = InitPQR2(1);
state2.Q = InitPQR2(2);
state2.R = InitPQR2(3);

