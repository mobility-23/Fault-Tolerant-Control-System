function [cineq,ceq] = nonlcon_function(vars, mpc_params, con_params, d_GP, mu_x0, angular_v_des_seq)
    % function [cineq,ceq,gradvars_cineq,gradvars_ceq] = nonlcon(obj, vars, t0, x0)
    %------------------------------------------------------------------
    % Evaluate nonlinear equality and inequality constraints
    % out:
    %   cineq = g(x,u) <= 0 : inequality constraint function
    %   ceq   = h(x,u) == 0 : equality constraint function
    %   gradx_cineq(x,u): gradient of g(x,u) w.r.t. x
    %   gradx_ceq(x,u):   gradient of h(x,u) w.r.t. x
    %------------------------------------------------------------------
    
    n = mpc_params.state_size;
    N = mpc_params.N;
    contraint_p = 0.95; %SMPC constraint probability
    % init outputs
    ceq = [];
    cineq = [];
    % split variables
    u_vec = vars;
    var_x0 = zeros(n);
    % calculate state sequence for given control input sequence and x0
    [mu_xvec,var_xvec] = predict_state_sequence(d_GP, mu_x0, var_x0, u_vec, mpc_params, angular_v_des_seq);
    
    % provided equality constraints(h==0)
    h  = @(x,u) [];
    g  = @(x_mu,x_var,u) [];
    %g  = @(x_mu,x_var,u) tighten_ineq_constraints(mpc_params,con_params,x_mu,x_var,v);
    m = mpc_params.control_size;
    nh = length(h(zeros(n,1),zeros(m,1)));
    ng = length(g(zeros(n,1),zeros(n),zeros(m,1)));
    ceq_h   = zeros(nh, N);
    cineq_g = zeros(ng, N);
    for iN=1:N
        ceq_h(:,iN) = h(mu_xvec(:,iN),u_vec(iN));
        cineq_g(:,iN) = g(mu_xvec(:,iN+1),var_xvec(:,:,iN+1),u_vec(iN));
    end
    
    % provided inequality constraints (g<=0)
    % tighten constraints value gamma
    % state
    
    ceq   = ceq_h(:);
    cineq = cineq_g(:);
end


    
    
    