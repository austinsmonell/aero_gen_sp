function nlobj = generate_multi_mpc(x_init, u, param)


%%
% Specify the controller sample time, prediction horizon, and control
% horizon. 
Ts = 0.05;
Tstop = 9;

%% Planner

ctr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
% x_plan(1, :) = [x_init([1:3, 13:15]), (eul2rotm([x_init(9), x_init(8), x_init(7)])*normalize([x_init(4), x_init(5), x_init(6)], 'norm')')', (eul2rotm([x_init(21), x_init(20), x_init(19)])'*normalize([x_init(16), x_init(17), x_init(18)], 'norm')')']';
x_plan(1, :) = x_init([1:3,13:15,7:9,  19:21]);

x = x_init;
for i = 1:Tstop/Ts
%[ctr,simdata_plan,info] = nlmpcmove(nlobj_plan,x,ctr, simdata_plan);
[~,X] = ode113(@(t,x) get_full_state(x,ctr, param),[0 Ts],x);
x = X(end, :);
% x_plan(i+1, :) = [x([1:3, 13:15]), (eul2rotm([x(9), x(8), x(7)])*normalize([x(4), x(5), x(6)], 'norm')')', (eul2rotm([x(21), x(20), x(19)])*normalize([x(16), x(17), x(18)], 'norm')')']';
x_plan(i+1, :) = x([1:3,13:15,7:9,  19:21]);

t = i*Ts;

figure(1);
hold on;
plot3(x(1, 1), x(1, 2), x(1, 3), '-o','Color', 'm');
plot3(x(1, 13), x(1, 14), x(1, 15), '-o','Color', 'y');
end

%% Rotation
el = deg2rad(-3);
az = deg2rad(10.8);
r = 138;
R_el = [cos(el), 0, sin(el); 0, 1, 0; -sin(el), 0, cos(el)];
R_az_1 = [cos(az), -sin(az), 0; sin(az), cos(az), 0; 0, 0, 1];
R_az_2 = [cos(-az), -sin(-az), 0; sin(-az), cos(-az), 0; 0, 0, 1];

x_plan(:, 1:3) = (R_az_1'*R_el'*x_plan(:, 1:3)')';
% x_plan(:, 7:9) = (R_az_1'*R_el'*x_plan(:, 7:9)')';
x_plan(:, 4:6) = (R_az_2'*R_el'*x_plan(:, 4:6)')';
% x_plan(:, 10:12) = (R_az_2'*R_el'*x_plan(:, 10:12)')';
x_plan(:, 1) = x_plan(:, 1)-r;
x_plan(:, 4) = x_plan(:, 4)-r;

tilt_long = deg2rad(-5);
tilt_lat = deg2rad(0);
R_y = [cos(tilt_long), 0, sin(tilt_long); 0, 1, 0; -sin(tilt_long), 0, cos(tilt_long)];
R_z = [cos(tilt_lat), -sin(tilt_lat), 0; sin(tilt_lat), cos(tilt_lat), 0; 0, 0, 1];
x_plan(:, 1:3) = (R_y*R_z*x_plan(:, 1:3)')';
x_plan(:, 4:6) = (R_y*R_z*x_plan(:, 4:6)')';
% x_plan(:, 7:9) = (R_y*R_z*x_plan(:, 7:9)')';
% x_plan(:, 10:12) = (R_y*R_z*x_plan(:, 10:12)')';

x_plan(:, 1) = x_plan(:, 1)+r;
x_plan(:, 4) = x_plan(:, 4)+r;

x_plan(:, 1:3) = (R_el*R_az_1*x_plan(:, 1:3)')';
% x_plan(:, 7:9) = (R_el*R_az_1*x_plan(:, 7:9)')';
x_plan(:, 4:6) = (R_el*R_az_2*x_plan(:, 4:6)')';
% x_plan(:, 10:12) = (R_el*R_az_2*x_plan(:, 10:12)')';

for i = 1:Tstop/Ts+1
x_plan(i, 4:6) = normalize(x_plan(i, 4:6), 'norm').*(300.014-(sqrt(x_plan(i, 1).^2+x_plan(i, 2).^2+x_plan(i, 3).^2)));
end

figure(1);
hold on;
plot3(x_plan(:, 1),x_plan(:, 2), x_plan(:, 3),'Color', 'b')
plot3(x_plan(:, 4),x_plan(:, 5), x_plan(:, 6),'Color', 'k')

% len = 50;
% for i = 1:(Tstop/Ts+1)
% plot3([x_plan(i, 1),x_plan(i, 1)+x_plan(i, 7)*len] ,[x_plan(i, 2),x_plan(i, 2)+x_plan(i, 8)*len], [x_plan(i, 3),x_plan(i, 3)+x_plan(i, 9)*len],'Color', 'c')
% plot3([x_plan(i, 4),x_plan(i, 4)+x_plan(i, 10)*len],[x_plan(i, 5),x_plan(i, 5)+x_plan(i, 11)*len], [x_plan(i, 6),x_plan(i, 6)+x_plan(i, 12)*len],'Color', 'm')
% end


%% Optimize
%% 
% Create a nonlinear MPC controller with a prediction model
p = 15;
nlobj = nlmpcMultistage(p, 26,12);

%%
% Specify the controller sample time, prediction horizon, and control
% horizon. 
nlobj.Ts = Ts;


nlobj.UseMVRate = true;


nlobj.Model.StateFcn = @get_full_state;
nlobj.Model.ParameterLength = 41;

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
% nlobj.MV(5).Min = -0;
% nlobj.MV(5).Max = 0;
% nlobj.MV(6).Min = -.00;
% nlobj.MV(6).Max = .00;
% nlobj.MV(7).Min = -0;
% nlobj.MV(7).Max = 0;
% nlobj.MV(8).Min = -0;
% nlobj.MV(8).Max = 0;

nlobj.MV(1).Min = -0.6;
nlobj.MV(1).Max = .6;
nlobj.MV(2).Min = -.6;
nlobj.MV(2).Max = .6;
nlobj.MV(3).Min = -.6;
nlobj.MV(3).Max = .6;
nlobj.MV(4).Min = -.6;
nlobj.MV(4).Max = .6;
nlobj.MV(5).Min = -.6;
nlobj.MV(5).Max = .6;
nlobj.MV(6).Min = -.6;
nlobj.MV(6).Max = .6;
nlobj.MV(7).Min = -5;
nlobj.MV(7).Max = 5;
nlobj.MV(8).Min = -5;
nlobj.MV(8).Max = 5;

%% Stages
for ct = 1:p+1
nlobj.Stages(ct).CostFcn ='cost_func';
%nlobj.Stages(ct).CostJacFcn = 'cost_func_grad';
nlobj.Stages(ct).ParameterLength = 12;
end

%% Run
simdata = getSimulationData(nlobj);
simdata.StateFcnParameter = param';

validateFcns(nlobj,x_init,u, simdata);

ctr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
x = x_init;
prev_theta = x(25);
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
    plot3(x(1, 13), x(1, 14), x(1, 15), '-o','Color', 'g')
     
    figure(2);
    hold on;
    scatter(t, ctr(1), 'r');
    scatter(t, ctr(2), 'g');
    scatter(t, ctr(3), 'b');
    scatter(t, ctr(4), 'y');
    scatter(t, ctr(5), 'c');
    scatter(t, ctr(6), 'm');
    scatter(t, ctr(7), 'k');
    scatter(t, ctr(8), 'k');
    legend('da1', 'de1', 'dr1','da2',  'de2', 'dr2', 'dt1', 'dt2');
    xlabel('Time [s]');
    ylabel('control [%]');
    
    figure(3);
    hold on;
    scatter(t,info.Cost)
    title('Cost');
   

    j = abs((x(25)-prev_theta)*x(26)*0);
    w = j/Ts;
    figure(6);
    hold on;
    scatter(t, w);
    prev_theta = x(25); 
    title("Power W")

    drawnow;

end

end