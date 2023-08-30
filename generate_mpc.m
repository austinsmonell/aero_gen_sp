function nlobj = generate_mpc(ac1_state, ac2_state, gen_state, ctr1, ctr2, const, coeff,coeff_ctr, yaw_moment_1, yaw_moment_2, pitch_c, roll_c_1, roll_c_2)
%% Overview of Simulink Model
% Open the Simulink model.
mdl = 'aero_gen';
open_system(mdl)

%% 
% Create a nonlinear MPC controller with a prediction model
nlobj = nlmpc(26, 12,6);

%%
% Specify the controller sample time, prediction horizon, and control
% horizon. 
nlobj.Ts = 0.02;
nlobj.PredictionHorizon = 5;
nlobj.ControlHorizon = 5;



%% 
% Specify the state function for the nonlinear plant model and its
% Jacobian.


nlobj.Model.StateFcn = @(y,u) get_full_state(y, [u(1), u(2), u(3)], [u(4) , u(5), u(6)], const, coeff,coeff_ctr, yaw_moment_1, yaw_moment_2, pitch_c, roll_c_1, roll_c_2);


%nlobj.Jacobian.StateFcn = @(x,u) LaneFollowingStateJacFcn(x,u);

%% 
% Specify the output function for the nonlinear plant model and its
% Jacobian. The output variables are:


% nlobj.Model.OutputFcn = @(y,u) [(sqrt(y(1)^2 + y(2)^2 + y(3)^2)-150), (sqrt(y(13)^2 + y(14)^2 + y(15)^2)-150)]';
nlobj.Model.OutputFcn = @(y,u) [y(1),y(2), y(3), y(13), y(14), y(15), (eul2rotm([y(9), y(8), y(7)])'*normalize([y(4), y(5), y(6)], 'norm')')', (eul2rotm([y(21), y(20), y(19)])'*normalize([y(16), y(17), y(18)], 'norm')')']';

%nlobj.Jacobian.OutputFcn = @(x,u) [0 0 1 0 0 0 0;0 0 0 0 1 0 0;0 0 0 0 0 1 1];

%% 
% Set the constraints for manipulated variables.
nlobj.MV(1).Min = -0.6;
nlobj.MV(1).Max = 0.6;
nlobj.MV(2).Min = -0.6;
nlobj.MV(2).Max = 0.6;
nlobj.MV(3).Min = -0.6;
nlobj.MV(3).Max = 0.6;
nlobj.MV(4).Min = -0.6;
nlobj.MV(4).Max = 0.6;
nlobj.MV(5).Min = -0.6;
nlobj.MV(5).Max = 0.6;
nlobj.MV(6).Min = -0.6;
nlobj.MV(6).Max = 0.6;


%% 
% Set the scale factors.
nlobj.OV(1).ScaleFactor = 1;
nlobj.OV(2).ScaleFactor = 1;
nlobj.OV(3).ScaleFactor = 1;
nlobj.OV(4).ScaleFactor = 1;
nlobj.OV(5).ScaleFactor = 1;
nlobj.OV(6).ScaleFactor = 1;
nlobj.OV(7).ScaleFactor = 1;
nlobj.OV(8).ScaleFactor = 1;
nlobj.OV(9).ScaleFactor = 1;
nlobj.OV(10).ScaleFactor = 1;
nlobj.OV(11).ScaleFactor = 1;
nlobj.OV(12).ScaleFactor = 1;

nlobj.MV(1).ScaleFactor = 1;
nlobj.MV(2).ScaleFactor = 1;
nlobj.MV(3).ScaleFactor = 1;
nlobj.MV(4).ScaleFactor = 1;
nlobj.MV(5).ScaleFactor = 1;
nlobj.MV(6).ScaleFactor = 1;

%% 
% Specify the weights in the standard MPC cost function.
nlobj.Weights.OutputVariables = [1, 1, 1, 1, 1, 1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5];

%%
% Penalize acceleration change more for smooth driving experience.
nlobj.Weights.ManipulatedVariablesRate = [3, 3, 3, 3, 3, 3];

%% 
% Validate prediction model functions at an arbitrary operating point using
% the |validateFcns| command. At this operating point:
%
% * |x0| contains the state values.
% * |u0| contains the input values.
% * |ref0| contains the output reference values.
% * |md0| contains the measured disturbance value.
%
x0 = [ac1_state, ac2_state, gen_state];
u0 = [0,0, 0, 0, 0, 0];
ref0 = [0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
validateFcns(nlobj,x0,u0,[],[],ref0);

end