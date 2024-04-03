function [steer_cmd, acc, state_nom, matrix_state_new,... 
          angular_v_des,... 
          P_kminl, P_kminlnom,r_k_hat]                               =  calc_mpc(trajref, delta_x,...
                                                                        veh_pose, ref_pose, mpc_params,... 
                                                                        index, veh_params, matrix_state,...
                                                                        angular_v_des, P_kminl, P_kminlnom,...
                                                                        log, i)
%% Update state
basic_state_size   = 6;
basic_control_size = 2;
N                  = mpc_params.N;
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

v                    = veh_params.velocity;  
v_des                = trajref(index,7);     
angular_v            = veh_params.angular_v; 
angular_v_des        = v_des * k; 
angular_v_des_seq    = k_seq * v_des;

%% Update matrix
mpc_matrix = set_mpc_matrix(veh_params, v);
A                 = mpc_matrix.Ac;
B                 = mpc_matrix.Bc;
C                 = mpc_matrix.Cc;
I                 = mpc_params.I;
ts                = mpc_params.Ts;

[Ad, Bd, Cd] = matrix_discretization(A, B, C, ts);
                
%% Feed forward angle Update
kv = veh_params.lr*veh_params.mass/2/veh_params.Calfa_front/...
     veh_params.wheel_base - veh_params.lf*veh_params.mass...
     /2/veh_params.Calfa_rear/veh_params.wheel_base;
steer_feedforward = atan(veh_params.wheel_base*k + kv *v*v*k);
steer_feedforward = angle_normalization(steer_feedforward);
  
%% apply MPC solver
horizon = 10;   
control = 2;    
lower_bound=zeros(control,1);
lower_bound(1,1)=-veh_params.max_steer_angle;
lower_bound(2,1)=-veh_params.max_deceleration;
upper_bound=zeros(control,1);
upper_bound(1,1)=veh_params.max_steer_angle;
upper_bound(2,1)=veh_params.max_acceleration;
matrix_q         = mpc_params.Q;
matrix_r         = mpc_params.R;
ref_state=zeros(basic_state_size, 1);

% QP
command = solve_mpc_problem_QP(Ad,Bd,Cd...
          ,matrix_q,matrix_r,lower_bound...
          ,upper_bound,ref_state,horizon,control,...
          matrix_state, angular_v_des_seq );

steer_cmd = command(1); 
acc = command(2);

%% Update state
state_nom_prior = Ad * matrix_state + Bd * command + Cd * angular_v_des;
state_nom_measure =   state_nom_prior;

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
command_new (1,:) = command_new(1,:) * 0.6;
matrix_state_est = Ad * matrix_state + Bd * command_new + Cd * angular_v_des; 
 
%% states under fautls
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



 
 
 


