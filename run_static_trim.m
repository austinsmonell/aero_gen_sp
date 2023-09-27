clc
clear
close all

%% Load Data
addpath('../aero_gen_library_sp/')

load("state1.mat");
load("wing1.mat");

%% System Parameters
viz_on = 1;
ac1_forces = 1;
ac1_moments = 1;

gravity = [0 , 0, 9.81];
rho = 1.14;
k_line = 100;
d =100;
WindVelocity = [6, 0, 0];%[12, 0, 0]

%% Aircraft Parameters
roll_moment_1 = 0;
pitch_moment_1 = 0;
yaw_moment_1 = 0.0;%+
cl_1 = 6.2;

roll_c_1 = deg2rad(12.6);%-5
pitch_c = deg2rad(-10);%-18
k_roll = 0;%100
k_pitch = 0;%1000

%% Constants Vector
coeff = cell2mat(wing1.Coefficients.Values);
coeff_ctr = cell2mat(wing1.Surfaces.Coefficients.Values);
param = [state1.Mass, rho, wing1.ReferenceArea, wing1.ReferenceLength, wing1.ReferenceSpan, gravity, WindVelocity,...
         state1.Inertia{1, 1}, state1.Inertia{2, 2}, state1.Inertia{3, 3}, k_roll, k_pitch, d, k_line,...
         coeff(1, 1), coeff(1, 3), coeff(2, 6), coeff(3, 1), coeff(3, 3), coeff(4, 8), coeff(4,8), coeff(5, 3), coeff(5, 5), coeff(6, 6), coeff(6, 9),...
         coeff_ctr(4, 1), coeff_ctr(5, 2), coeff_ctr(6, 3), yaw_moment_1, pitch_c, roll_c_1]';

%% Run Study
angles = [0:5:90]';
wing_loading = [1:5:61]';
[X, Y] = meshgrid(wing_loading, angles);
Z = -20*ones(size(X));
for i=1:size(wing_loading, 1)
    for j=1:size(angles, 1)

%% Trim Targets
param(1) = wing_loading(i)*param(3)/gravity(3);
el_t = deg2rad(angles(j));
az_t = deg2rad(0);

z_t = -sin(el_t)*d;
x_t = sin(az_t)*d;

%% Initial Guess
InitPosition1 = [sqrt(d^2-x_t^2-z_t^2); x_t; z_t];
InitVelocity1 = [0; 0; 0];
InitEuler1 = [0; pi/8; pi];
InitPQR1 = [0; 0; 0];

%% Create IC
x0 = [InitPosition1; InitVelocity1; InitEuler1; InitPQR1];
u0 = zeros(3, 1);

%% Static Trim
targets = [z_t, x_t];
[z_trim, f0] = static_trim(x0, u0, param, targets);
x_trim = z_trim(1:12);
u_trim = z_trim(13:15);

if f0<1e-4
    Z(j, i) = x_trim(8)*180/pi;
end

    end
    i
end

stop_time = 10;
tspan = [0 stop_time];
[t, x_out] = ode45(@(t, x) get_full_state(x, u_trim, param), tspan, x_trim);

%% Plot Trim
figure(2);
surf(X, Y, Z);
xlabel('WL [Pa]');
ylabel('Elevation Angle [deg]');
zlabel('Theta [deg]');
cb = colorbar("westoutside");
cb.Label.String = "Theta/AoA [deg]";



figure(1);
subplot(2, 2, 1)
hold on;
plot(t, x_out(:, 1))
plot(t, x_out(:, 2))
plot(t, x_out(:, 3))
legend('X', 'Y', 'Z')
subplot(2, 2, 2)
hold on;
plot(t, x_out(:, 4))
plot(t, x_out(:, 5))
plot(t, x_out(:, 6))
legend('U', 'V', 'W')
subplot(2, 2, 3)
hold on;
plot(t, x_out(:, 7))
plot(t, x_out(:, 8))
plot(t, abs(x_out(:, 9))-pi)
legend('Phi', 'Theta', 'Psi')
subplot(2, 2, 4)
hold on;
plot(t, x_out(:, 10))
plot(t, x_out(:, 11))
plot(t, x_out(:, 12))
legend('P', 'Q', 'R')



%% Run Simulink

InitPosition1 = x_trim(1:3);
InitVelocity1 = x_trim(4:6);
InitEuler1 = x_trim(7:9);
InitPQR1 = x_trim(10:12);

%% Set Parameters
[state1] = set_parameters(WindVelocity,InitPosition1, InitVelocity1, InitEuler1, InitPQR1, state1);


out = sim('aero_gen.slx');