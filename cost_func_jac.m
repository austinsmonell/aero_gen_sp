function [Gx, Gmv, Gdmv] = cost_func_jac(stage, x, u,dmv, param)
err = [x(1:6)-param; zeros(6, 1)];
Q = [1, 0, 0, 0, 0, 0;...
     0, 1, 0, 0, 0, 0;...
     0, 0, 1, 0, 0, 0;...
     0, 0, 0, 1, 0, 0;...
     0, 0, 0, 0, 1, 0;...
     0, 0, 0, 0, 0, 1]
     Q = zeros(12);
     Q([1:6], [1:6]) = eye(6);
     Q(1, 1) = 1;
Gmv = zeros(4,1);
Gx = 2*Q*err;
Gdmv = 2*0.01*dmv;

end