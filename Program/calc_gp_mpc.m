function [command, matrix_state_new, state_nom, state_est,... 
          state_true, uguess_new, angular_v_des,...
          P_kminl, P_kminlnom,r_k_hat] =                      calc_gp_mpc(trajref, delta_x, veh_pose,... 
                                                              mpc_params,index, veh_params,...
                                                              matrix_state, con_params,...
                                                              true_mpc_params, uguess, d_GP,...
                                                              UseParallel, maxiter, angular_v_des,...
                                                              P_kminl, P_kminlnom,...
                                                              log, i)
%% Reference angular velocity
N                 = mpc_params.N;
k                 = trajref(index,5);
num               = veh_params.Vx * mpc_params.Ts / 0.2;
% k_seq             = trajref(index:num:index + num *(N-1),5);
k_seq             = trajref(index:index+N-1,5); 
v                 = veh_params.velocity;  
v_des             = trajref(index,7);    
angular_v         = veh_params.angular_v; 
angular_v_des     = v_des * k; 
angular_v_des_seq = k_seq * v_des;

%% Update parameter
dx               = delta_x(1);
dy               = delta_x(2);
dtheta           = delta_x(3);
theta_des        = trajref(index,3);
theta            = veh_pose(3);
radius_des       = trajref(index,4);
k                = trajref(index,5);  
k_seq            = trajref(index:index+N-1,5); 

one_min_k        = 1-k;
if one_min_k<=0
    one_min_k=0.01;
end

%% Update matrix
mpc_matrix = set_mpc_matrix(veh_params, v);
A                 = mpc_matrix.Ac;
B                 = mpc_matrix.Bc;
C                 = mpc_matrix.Cc;
I                 = mpc_params.I;
ts                = mpc_params.Ts;
[Ad, Bd, Cd] = matrix_discretization(A, B, C, ts);
Dd = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];
mpc_params.Ad = Ad;
mpc_params.Bd = Bd;
mpc_params.Cd = Cd;
mpc_params.Dd = Dd;

%% fmincon solver
[u_opt, uguess_new] = optimization_solve_fmincon(mpc_params, con_params,...
    matrix_state, uguess, d_GP, UseParallel, maxiter, angular_v_des_seq);

command = u_opt(:,1);
steer_cmd = u_opt(1,1);
acc_cmd = u_opt(2,1);

%% Update state
state_nom_prior = Ad * matrix_state + Bd * command + Cd * angular_v_des;
state_nom_measure = state_nom_prior;

%% EKF filter 
angular_v_des_KF = angular_v_des;
[state_nom_est,P_knom] = Kalmanfilter_hhy_modified_1(mpc_params,...
                                                                       state_nom_measure,...
                                                                       state_nom_prior,...
                                                                       angular_v_des_KF,...
                                                                       P_kminlnom,...
                                                                       i); 
state_nom = state_nom_est;
P_kminlnom = P_knom;

%% Faults                                                               
command_new = command;
command_new (1,:) = command_new(1,:) * 0.8;
matrix_state_est = Ad * matrix_state + Bd * command_new + Cd * angular_v_des;
 
%% State under faults
state_kine_state_est = Ad * matrix_state + Bd * command_new + Cd * angular_v_des; 
matrix_state_measure = state_kine_state_est;

%% EKF filter
angular_v_des_KF = angular_v_des;
[matrix_state_est,P_k,r_k_hat] = Kalmanfilter_hhy_modified(mpc_params,...
                                                                       matrix_state_measure,...
                                                                       matrix_state_est,...
                                                                       state_nom,...
                                                                       angular_v_des_KF,...
                                                                       P_kminl,...
                                                                       i); 
matrix_state_new = matrix_state_est;
P_kminl = P_k;
 
D_x = [1 0 0 0 0 0;0 0 1 0 0 0];
D_u = [1 0;0 1];
z = [D_x * matrix_state;
     D_u * command];

[mean_gp,var_gp] = d_GP.eval(z,true);
state_est = state_nom - Dd * ( mean_gp );
state_true = matrix_state_new;

%% Fault detection 
%MSIS_inf detector
% [fault_detection_MSIS_inf,test_MSIS_inf,threshold_MSIS_inf] = MSIS_detecor_inf(r_k_hat,log,i);

end
