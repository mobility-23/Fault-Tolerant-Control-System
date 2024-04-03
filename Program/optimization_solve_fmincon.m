%uguess = zeros(m,N);

function [u_opt, uguess] = optimization_solve_fmincon(mpc_params,...
    con_params, x0, uguess, d_GP, UseParallel, maxiter, angular_v_des_seq)
    %------------------------------------------------------------------
    % Calculate first uptimal control input
    %------------------------------------------------------------------
    varsguess = uguess(:); 
    costfun = @(vars) cost_function(vars, mpc_params, d_GP, x0, angular_v_des_seq);
    %nonlcon = @(vars) nonlcon_function(vars, mpc_params, con_params, d_GP, x0, angular_v_des_seq);
    u_ll = con_params.u_ll;
    u_uu = con_params.u_uu;
    N = mpc_params.N;
    lb = repmat(u_ll,N,1);
    ub = repmat(u_uu,N,1);

    options = optimoptions('fmincon',...
        'Display','iter',...
        'Algorithm', 'interior-point',... % 'interior-point',... % 'sqp','interior-point'
        'SpecifyConstraintGradient',false,...
        'UseParallel',UseParallel,... %'ConstraintTolerance',obj.tol,...
        'MaxIterations',maxiter);
    [vars_opt,~] = fmincon(costfun,varsguess,[],[],[],[],lb,ub,[],options);
    %[vars_opt,~] = fmincon(costfun,varsguess,[],[],[],[],lb,ub,nonlcon,options);
    
    u_opt = reshape(vars_opt,mpc_params.control_size,N);
    uguess = u_opt(:,[2:end,end]);
end
            
    