function [mu_xkp1,var_xkp1] = state_prediction (d_GP, mpc_params, mu_xk, var_xk, uk,  index, angular_v_des_seq)

[fd, gradx_fd] = fd_nom(mpc_params,mu_xk,uk, index, angular_v_des_seq);
Dd = mpc_params.Dd;
grad_xkp1 = [gradx_fd; Dd'; Dd'];

z = generate_z(mu_xk,uk);

[mu_d, var_d] = d_GP.eval(z,true);

var_w = zeros(2);
var_x_d_w = blkdiag(var_xk, var_d, var_w);

mu_xkp1  = fd  + Dd * (mu_d);
var_xkp1 = grad_xkp1' * var_x_d_w * grad_xkp1; % zeros(obj.n);
end
