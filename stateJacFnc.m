function [A, B] = stateJacFnc(x_in, u_in, param)

x = struct('f',x_in,'dx',ones(12));
y = deriv2(x,u_in, param);
A = sparse(y.dx_location(:,1),y.dx_location(:,2),y.dx,12,12)+zeros(12);

x = struct('f',u_in,'dx',ones(12, 4));
y = deriv_ctr(x_in, x, param);
B = sparse(y.dx_location(:,1),y.dx_location(:,2),y.dx,12,4)+zeros(12, 4);

end
