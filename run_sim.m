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
rho = 1.225;
k_line = 100;
CD_tether = 1;
d =80;
S_tether = d*0.001;
WindVelocity = [5, 0, 0];%[12, 0, 0]

%% Aircraft Parameters
roll_moment_1 = 0;
pitch_moment_1 = 0;
yaw_moment_1 = 0.00;%+

roll_c = deg2rad(0);%-5
pitch_c = deg2rad(3);%-18
k_roll = 500;%100
k_pitch = 1000;%1000

%% Control Law Parameters
trg_el = 40;
trg_az = 45;
motor_kv = 30;
motor_kt = 60/(2*pi*motor_kv);
motor_moi = 0.0003;
el_bias = 0;
az_bias = 0;

%% Constants Vector
coeff = cell2mat(wing1.Coefficients.Values);
coeff_ctr = cell2mat(wing1.Surfaces.Coefficients.Values);

param = [state1.Mass, rho, wing1.ReferenceArea, wing1.ReferenceLength, wing1.ReferenceSpan, gravity, WindVelocity,...
         state1.Inertia{1, 1}, state1.Inertia{2, 2}, state1.Inertia{3, 3}, k_roll, k_pitch, d, k_line,...
         coeff(1, 1), coeff(1, 3), coeff(2, 6), coeff(3, 1), coeff(3, 3), coeff(4, 8), coeff(4,8), coeff(5, 3), coeff(5, 5), coeff(6, 6), coeff(6, 9),...
         coeff_ctr(4, 1), coeff_ctr(5, 2), coeff_ctr(6, 3), yaw_moment_1, pitch_c, roll_c];

%% Initial Conditions
% 
% 
InitPosition1 = [0, 0.0001, -100]';
InitVelocity1 = [0, 0.0001, 0.0001]';
InitAcc1 = [0, 0, 0]';
InitEuler1 = [0, 0, 179.99]'*pi/180;
InitPQR1 = [0, 0, 0]';
InitPQRDot1 = [0, 0, 0]';
% 

%% Load Initial Conditions
% 
% load("Flight Configurations\IC_Saves\150m_SS_1.mat");
% 
% InitPosition1 = IC_ss(1:3);
% InitVelocity1 = IC_ss(4:6);
% InitAcc1 = IC_ss(7:9);
% InitEuler1 = IC_ss(10:12);
% InitPQR1 = IC_ss(13:15);
% InitPQRDot1 = IC_ss(16:18);

%% Static Trim
x0 = [InitPosition1; InitVelocity1; InitEuler1; InitPQR1];
u0 = zeros(3, 1);
x_trim = x0;
u_trim = u0;

targets = [0];
[z_trim, f0] = static_trim_full(x0, u0, param, targets);
x_trim = [z_trim(1); x0(2); z_trim(2);x0(4:7); z_trim(3); x0(9:12)];
u_trim = [u0(1); z_trim(4); u0(3)];

InitPosition1 = x_trim(1:3);
InitVelocity1 = x_trim(4:6);
InitEuler1 = x_trim(7:9);
InitPQR1 = x_trim(10:12);

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

%% Run Sim
run_sim_on = 0;
if run_sim_on == 1
    tic
    stop_time = 30;
    out = sim('aero_gen.slx');
    toc
end


%% Plot Output
plot_sim_on = 0;
if plot_sim_on == 1
    time = out.tout;
    y = out.state.signals.values;
    
    %3d plot flightpath
    figure(1);
    hold on;
    plot3(y(:, 1), y(:, 2), -y(:, 3));
    grid on;
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
    
    %psi
    figure(5);
    plot(time, y(:, 19)*180/pi);
    hold on;
    xlabel('Time [s]');
    ylabel('Psi [def]');
    title('Psi vs Time');
    
    %% Test Get State Function
    
    tic;
    [t, y_out] = run_sim_states(stop_time, x_trim, u_trim, param);
    toc
    
    %Plot from sim states
    %3d plot
    figure(1);
    hold on;
    plot3(y_out(:, 1), y_out(:, 2), -y_out(:, 3));
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title('Flight Path [1 Cycle]')
    %radius
    figure(4);
    plot(t, sqrt(y_out(:, 1).^2 + y_out(:, 2).^2 + y_out(:, 3).^2));
    xlabel('Time [s]');
    ylabel('Radius [m]');
    title('Radius vs Time');
end

monte_carlo = 0;
if monte_carlo
    stop_time = 60;
    cn_beta = [10, 5, 1, 0.5, 0.2];
%     cL_alpha = [3, 4, 5, 6, 7];
%     cD_alpha = [0.05, 0.1, 0.2, 0.3, 0.4];
%     az_bias_vec = [0, 1, 3, 6, 10]*pi/180;
%     el_bias_vec = [0, 1, 3, 6, 10]*pi/180;
    for i = 1:1
        wing1.Coefficients.Table.Beta(6)=cn_beta(i);
%         wing1.Coefficients.Table.Alpha(3)=cL_alpha(i);
%         wing1.Coefficients.Table.Alpha(1)=cD_alpha(i);
%         az_bias = az_bias_vec(i);
%         el_bias = el_bias_vec(i);
        out = sim('aero_gen.slx');
        log_idx = find(out.state.time > 35);
        xyz = out.state.signals.values(log_idx, 1:3);
        plot3(xyz(:, 1), -xyz(:, 2), -xyz(:, 3), 'LineWidth',3);
        title("Flight Path")
        xlabel("X [m]");
        ylabel("Y [m]");
        zlabel("Z [m]");
        hold on;
        axis equal;
        drawnow;
    end
%     legend("Cn-beta="+num2str(cn_beta(1)),"Cn-beta="+num2str(cn_beta(2)), "Cn-beta="+num2str(cn_beta(3)), "Cn-beta="+num2str(cn_beta(4)), "Cn-beta="+num2str(cn_beta(5)))
%     legend("CL-alpha="+num2str(cL_alpha(1)),"CL-alpha="+num2str(cL_alpha(2)), "CL-alpha="+num2str(cL_alpha(3)), "CL-alpha="+num2str(cL_alpha(4)), "CL-alpha="+num2str(cL_alpha(5)))
%     legend("CD-alpha="+num2str(cD_alpha(1)),"CD-alpha="+num2str(cD_alpha(2)), "CD-alpha="+num2str(cD_alpha(3)), "CD-alpha="+num2str(cD_alpha(4)), "CD-alpha="+num2str(cD_alpha(5)))
%     az_bias_vec = az_bias_vec*180/pi;legend("AZ Bias="+num2str(az_bias_vec(1)),"AZ Bias="+num2str(az_bias_vec(2)), "AZ Bias="+num2str(az_bias_vec(3)), "AZ Bias="+num2str(az_bias_vec(4)), "AZ Bias="+num2str(az_bias_vec(5)));
%     el_bias_vec = el_bias_vec*180/pi;legend("EL Bias="+num2str(el_bias_vec(1)),"EL Bias="+num2str(el_bias_vec(2)), "EL Bias="+num2str(el_bias_vec(3)), "EL Bias="+num2str(el_bias_vec(4)), "EL Bias="+num2str(el_bias_vec(5)));
end