function [fd, gradx_fd] = fd_nom(mpc_params,xk,uk, index, angular_v_des_seq)
    Ad = mpc_params.Ad;
    Bd = mpc_params.Bd;
    Cd = mpc_params.Cd;
    % fd
    fd = Ad * xk + Bd * uk + Cd * angular_v_des_seq(index);
    % gradx_fd
    gradx_fd = Ad;
end