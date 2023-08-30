
p = 5;
t = 0;
ctr = [0, 0, 0, 0, 0, 0];
step = 1:p;


simdata = getSimulationData(nlobj);
simdata.StateFcnParameter = param';
x = [ac1_state, ac2_state, gen_state];
for i = 1:500
i
param_ref = [];
for stg = 1:p
    gamma_offset = 0.0+(.2-0.0)*((stg-1)/(p-1));
    [gamma1, gamma2] = get_lambda(x, ref1, ref2);
    [ref_posi1, ~] = get_interp_posi(gamma1-gamma_offset, flight_path_1);
    [ref_posi2, ~] = get_interp_posi(gamma2+gamma_offset, flight_path_2);
    param_ref = [param_ref, param, ref_posi1, ref_posi2];
    figure(1);
hold on;
scatter3(ref_posi1(1), ref_posi1(2), ref_posi1(3), 'r');
scatter3(ref_posi2(1), ref_posi2(2), ref_posi2(3), 'b');
end

simdata.StageParameter = param_ref';

[mv,~,info] = nlmpcmove(nlobj,x,ctr, simdata);


% [~,X] = ode45(@(t,x) get_full_state(x,info.MVopt(1, :)', param),[0 .05],x);
% x = X(end, :);

x = info.Xopt(end, 1:26);



figure(1);
hold on;
plot3(info.Xopt(step, 1), info.Xopt(step, 2), info.Xopt(step, 3), '-o')
plot3(info.Xopt(step, 13), info.Xopt(step, 14), info.Xopt(step, 15), '-o')

% plot3(X(:, 1), X(:, 2), X(:, 3));
% plot3(X(:, 13), X(:, 14), X(:, 15));

figure(2);
hold on;
% scatter(t, info.MVopt(end-1, 1), 'r');
% scatter(t, info.MVopt(end-1, 2), 'g');
% scatter(t, info.MVopt(end-1, 3), 'b');
% scatter(t, info.MVopt(end-1, 4), 'y');
% scatter(t, info.MVopt(end-1, 5), 'c');
% scatter(t, info.MVopt(end-1, 6), 'm');
scatter(t, mv(1), 'r');
scatter(t, mv(2), 'g');
scatter(t, mv(3), 'b');
scatter(t, mv(4), 'y');
scatter(t, mv(5), 'c');
scatter(t, mv(6), 'm');
legend('da1', 'de1', 'dr1','da2',  'de2', 'dr2');
xlabel('Time [s]');
ylabel('control [%]');

figure(3);
hold on;
scatter(t,info.Cost)
drawnow;

t = t+0.1;
ctr = mv;



end


