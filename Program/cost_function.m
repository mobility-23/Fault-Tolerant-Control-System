function cost = cost_function(vars, mpc_params, d_GP, mu_x0, angular_v_des_seq) 

    n = mpc_params.state_size;
    N = mpc_params.N;
    u_vec = vars;
    var_x0 = zeros(n);
    [mu_xvec,var_xvec] = predict_state_sequence(d_GP, mu_x0, var_x0, u_vec, mpc_params, angular_v_des_seq);

    cost = 0;
    cost_stage_state = zeros(N,1);
    cost_stage_control = zeros(N,1);

    Q = mpc_params.Q;
    R = mpc_params.R;

    for i = 1:N
        cost_stage_state(i,1) = mu_xvec(:,i)'* Q * mu_xvec(:,i);
        cost_stage_control(i,1) =  u_vec(2*i-1:2*i)'* R * u_vec(2*i-1:2*i);
        cost = cost + cost_stage_state(i,1) + cost_stage_control(i,1);
    end

    cost_terminal_state = mu_xvec(:,N+1)'* Q * mu_xvec(:,N+1) + trace(Q * var_xvec(:,:,N+1));

end