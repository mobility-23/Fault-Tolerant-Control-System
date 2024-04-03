function [mu_xk,var_xk] = predict_state_sequence(d_GP, mu_x0, var_x0, uk, mpc_params, angular_v_des_seq)

    n = mpc_params.state_size;
    N = mpc_params.N;
    mu_xk  = zeros(n,N+1);
    var_xk = zeros(n,n,N+1);
    mu_xk(:,1)    = mu_x0;
    var_xk(:,:,1) = var_x0;

    for iN=1:N      
        index = iN;
        [mu_xk(:,iN+1),var_xk(:,:,iN+1)] = state_prediction(d_GP, mpc_params, mu_xk(:,iN), var_xk(:,:,iN), uk(2*iN-1:2*iN), index, angular_v_des_seq);
    end
end