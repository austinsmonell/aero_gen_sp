function nlobj = generate_multi_mpc(x_init, u, param)


%%
% Specify the controller sample time, prediction horizon, and control
% horizon. 
Ts = 0.1;
Tstop = 9;

%% Planner

ctr = [0, 0, 0, 0]';
% x_plan(1, :) = [x_init([1:3, 13:15]), (eul2rotm([x_init(9), x_init(8), x_init(7)])*normalize([x_init(4), x_init(5), x_init(6)], 'norm')')', (eul2rotm([x_init(21), x_init(20), x_init(19)])'*normalize([x_init(16), x_init(17), x_init(18)], 'norm')')']';
x_plan(1, :) = x_init(1:6);

x = x_init;
for i = 1:Tstop/Ts
%[ctr,simdata_plan,info] = nlmpcmove(nlobj_plan,x,ctr, simdata_plan);
[~,X] = ode113(@(t,x) get_full_state(x,ctr, param),[0 Ts],x);
x = X(end, :);
% x_plan(i+1, :) = [x([1:3, 13:15]), (eul2rotm([x(9), x(8), x(7)])*normalize([x(4), x(5), x(6)], 'norm')')', (eul2rotm([x(21), x(20), x(19)])*normalize([x(16), x(17), x(18)], 'norm')')']';
x_plan(i+1, :) = x(1:6);

t = i*Ts;

figure(1);
hold on;
plot3(x(1, 1), x(1, 2), x(1, 3), '-o','Color', 'm');
end
xlim([125, 150]);
ylim([-45, 85]);
zlim([-80, 80]);
axis equal;

%% Rotation
% el = deg2rad(-3);
% az = deg2rad(10.8);
% r = 138;
% R_el = [cos(el), 0, sin(el); 0, 1, 0; -sin(el), 0, cos(el)];
% R_az_1 = [cos(az), -sin(az), 0; sin(az), cos(az), 0; 0, 0, 1];
% 
% x_plan(:, 1:3) = (R_az_1'*R_el'*x_plan(:, 1:3)')';
% x_plan(:, 1) = x_plan(:, 1)-r;
% 
% tilt_long = deg2rad(-5);
% tilt_lat = deg2rad(0);
% R_y = [cos(tilt_long), 0, sin(tilt_long); 0, 1, 0; -sin(tilt_long), 0, cos(tilt_long)];
% R_z = [cos(tilt_lat), -sin(tilt_lat), 0; sin(tilt_lat), cos(tilt_lat), 0; 0, 0, 1];
% x_plan(:, 1:3) = (R_y*R_z*x_plan(:, 1:3)')';
% 
% x_plan(:, 1) = x_plan(:, 1)+r;
% 
% x_plan(:, 1:3) = (R_el*R_az_1*x_plan(:, 1:3)')';

figure(1);
hold on;
plot3(x_plan(:, 1),x_plan(:, 2), x_plan(:, 3),'Color', 'b')

%% Optimize
%% 
% Create a nonlinear MPC controller with a prediction model
p = 5;
nlobj = nlmpcMultistage(p, 12,4);

%%
% Specify the controller sample time, prediction horizon, and control
% horizon. 
nlobj.Ts = Ts;


nlobj.UseMVRate = true;


nlobj.Model.StateFcn = @get_full_state;
nlobj.Model.StateJacFcn = @stateJacFnc;
nlobj.Model.ParameterLength = 35;

%% 
% Set the constraints for manipulated variables.
% nlobj.MV(1).Min = -0.00;
% nlobj.MV(1).Max = 0.00;
% nlobj.MV(2).Min = -0;
% nlobj.MV(2).Max = 0;
% nlobj.MV(3).Min = -.00;
% nlobj.MV(3).Max = .00;
% nlobj.MV(4).Min = -0.00;
% nlobj.MV(4).Max = 0.00;
% 
% nlobj.MV(1).Min = -0.6;
% nlobj.MV(1).Max = .6;
% nlobj.MV(2).Min = -.6;
% nlobj.MV(2).Max = .6;
% nlobj.MV(3).Min = -.6;
% nlobj.MV(3).Max = .6;
% nlobj.MV(4).Min = -.6;
% nlobj.MV(4).Max = .6;

%% Stages
for ct = 1:p+1
nlobj.Stages(ct).CostFcn = 'cost_func';
nlobj.Stages(ct).CostJacFcn = 'cost_func_jac';
nlobj.Stages(ct).ParameterLength = 6;
end

%% Run
simdata = getSimulationData(nlobj);
simdata.StateFcnParameter = param';

validateFcns(nlobj,x_init,u, simdata);

ctr = [0, 0, 0, 0]';
x = x_init;
for i = 1:Tstop/Ts-p

    stg_param = [];
    for ct = i:(i+p)
        stg_param = [stg_param, x_plan(ct, :)];
    end
    simdata.StageParameter = stg_param';
    
    
    [ctr,simdata,info] = nlmpcmove(nlobj,x,ctr, simdata);

    [~,X] = ode113(@(t,x) get_full_state(x,ctr, param),[0 Ts],x);
    x = X(end, :);
    t = i*Ts;
    
    figure(1);
    hold on;
    plot3(x(1, 1), x(1, 2), x(1, 3), '-o','Color', 'r')
     
    figure(2);
    hold on;
    scatter(t, ctr(1), 'r');
    scatter(t, ctr(2), 'g');
    scatter(t, ctr(3), 'b');
    scatter(t, ctr(4), 'y');
    legend('da', 'de', 'dr','dt');
    xlabel('Time [s]');
    ylabel('control [%]');
    title('Control');
    
    figure(3);
    hold on;
    scatter(t,info.Cost)
    title('Cost');

    pos_err = (X(1, 1:3)-x_plan(i, 1:3))';
    vel_err = (X(1, 4:6)-x_plan(i, 4:6))';

    figure(5);
    hold on;
    scatter(t, pos_err(1), 'c');
    scatter(t, pos_err(2), 'r');
    scatter(t, pos_err(3), 'g');
    title('Position Error');


    figure(6);
    hold on;
    scatter(t, vel_err(1), 'r');
    scatter(t, vel_err(2), 'g');
    scatter(t, vel_err(3), 'b');
    title('Velocity Error');

    drawnow;

end

end