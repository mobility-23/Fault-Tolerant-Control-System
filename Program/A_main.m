
clc
clear
close all

%% 0. Select solver
% path_tracking_alg    = 'Nominal_MPC';
% solver = 'quadprog_solver';
path_tracking_alg    = 'GP_MPC';
solver = 'fmincon';

%% 1-0. Initial values of system states
veh_pose             = [0, 0, 0];               
command              = [0;0];
simulation_time      = 0;                             
run_distance         = 0;                             
i                    = 0;
time                 = 0;
matrix_state         = [2;0;0;0;0;-2];
matrix_control       = [0;0];
uguess               = [0;0];

%% 1-2.  Initial values of fault detection-related states
%MSIS_inf
fault_detection_MSIS_inf      = 0;                  
detection_TN_MSIS_inf         = 0;                     
detection_TP_MSIS_inf         = 0;                    
detection_FN_MSIS_inf         = 0;                     
detection_FP_MSIS_inf         = 0;                    
accuracy_MSIS_inf             = 0;                     
            
%% 2. Load paramters
veh_params = load_veh_params();
sim_params = load_sim_params();
con_params = load_con_params(veh_params);
mpc_params = load_mpc_params(veh_params, sim_params);
true_mpc_params = load_true_mpc_params(veh_params, sim_params);

%% 3. Reference path
roadmap_name = 'big_circle';                    
[trajref_params, simulation_stop_y, simulation_stop_time] = set_trajref_params(roadmap_name, veh_params);         
trajref                                                   = generate_trajref(trajref_params,roadmap_name);      

%% 4. vehicle tracking performance.                   
[path_figure, steer_figure,... 
          acc_figure, lateral_error,...
          heading_error, station_error,...
          speed_error,...
          fault_detection_MSIS_inf_figure]    = draw_path_tracking(path_tracking_alg, roadmap_name,... 
                                                                 trajref, veh_pose, command,...
                                                                 veh_params, simulation_time,... 
                                                                 simulation_stop_time, matrix_state,... 
                                                                 matrix_control,...
                                                                 fault_detection_MSIS_inf);

%% 5. Configure storage
u0           = command;
zhat0        = [0;0;0];
time_step    = sim_params.Ts;
log          = log_init(time_step, simulation_stop_time,  matrix_state, zhat0, u0);

%% 6. Hypothetical sensor parameters
P_kminl = [1,0,0,0,0,0,0;
           0,1,0,0,0,0,0;
           0,0,1,0,0,0,0;
           0,0,0,1,0,0,0;
           0,0,0,0,1,0,0;
           0,0,0,0,0,1,0
           0,0,0,0,0,0,1];

P_kminlnom = [1,0,0,0,0,0,0;
           0,1,0,0,0,0,0;
           0,0,1,0,0,0,0;
           0,0,0,1,0,0,0;
           0,0,0,0,1,0,0;
           0,0,0,0,0,1,0
           0,0,0,0,0,0,1];

%% 
disp([path_tracking_alg,' simulation start!']);

%% 7. Simulation
while((simulation_time < simulation_stop_time) && (veh_pose(2) < simulation_stop_y))   
    tic;     
    i               = i + 1;                                          
    time = i;
    simulation_time = simulation_time + time_step;                    
    run_distance    = run_distance + veh_params.velocity * time_step;  
    log.time(i)     = simulation_time;                                
    log.dist(i)     = run_distance;                                        
    t1 = clock;      
    switch (path_tracking_alg)         
        case 'Nominal_MPC'
            angular_v_des = 0;                      
            [steer_cmd, acc, state_nom, matrix_state_new,... 
            angular_v_des,...
            P_kminl, P_kminlnom,r_k_hat]                     = ALG_MPC(veh_pose, trajref,...
                                                               mpc_params, veh_params,...
                                                               time_step, matrix_state,...
                                                               angular_v_des, P_kminl, P_kminlnom,...
                                                               log, i);      
            command(1) = steer_cmd;
            command(2) = acc;     
        case 'GP_MPC'
            GPfile = fullfile(pwd,'/GP_u_20230728_delta_gain_0.6fault.mat');
            load(GPfile);    
            fprintf('\nGP model loaded succesfuly\n\n')     
            m           = mpc_params.control_size;
            N           = mpc_params.N;
            uguess      = zeros(m,N); 
            UseParallel = true;
            maxiter     = 30;          
            angular_v_des = 0;
            [command, matrix_state_new, state_nom,...
            state_est, state_true, uguess_new,...
            angular_v_des, P_kminl, P_kminlnom, r_k_hat]        = ALG_GP_MPC(veh_pose, trajref,mpc_params, veh_params,...
                                                                  matrix_state,  con_params,true_mpc_params,...
                                                                  uguess,d_GP, UseParallel, maxiter,... 
                                                                  angular_v_des,  P_kminl, P_kminlnom,...
                                                                  log, i);
            uguess = uguess_new;  
            steer_cmd = command(1);
            acc = command(2);                                             
        otherwise
            disp('There is no this lateral control algorithm!');
            break;
    end
    steer_state = steer_cmd;
    t2 = clock;
    log.solver_run_time(i) = etime(t2,t1);
  
    switch (path_tracking_alg)
        case 'Nominal_MPC'
            veh_pose = update_veh_pose_V(matrix_state_new, trajref, run_distance);
        
        case 'GP_MPC'
            veh_pose = update_veh_pose_V(matrix_state_new, trajref, run_distance);
            
        otherwise
            disp('There is no this lateral control algorithm!');
            break;
    end
  
    [~, index]                = calc_nearest_point(veh_pose, trajref);
    ref_pose                  = calc_proj_pose(veh_pose(1:2), trajref(index, 1:3), trajref(index + 1, 1:3));
    log.ref_pose(i, :)        = ref_pose;                      
    log.steer_cmd(i)          = steer_state / pi * 180;      
    log.acc(i)                = acc;
    log.veh_pose(i, :)        = veh_pose;             	       
    log.delta_x(i, :)         = veh_pose - ref_pose;           
    log.angular_v_des_vec(i, :) =  angular_v_des;
    veh_params.angular_v = update_angular_velocity(log,time_step,i);
    veh_params.velocity  = veh_params.velocity + acc * time_step;    
    log.velocity(i, :) = veh_params.velocity;
    log.u(:,i)       = command;
    matrix_state     = matrix_state_new;
    log.xhat(:,i+1)  = matrix_state;
    log.xnom(:,i+1)  = state_nom;
    
    %% 8. Fault detection
%     log.MSIS_inf(i,:)  = test_MSIS_inf;   
%     log.fault_detection_MSIS_inf(i,:) = fault_detection_MSIS_inf;   
%     log.threshold_MSIS_inf(i,:)       = threshold_MSIS_inf;           

    %Accuracy
%     accuracy_MSIS_inf = 1-(detection_FN_MSIS_inf + detection_FP_MSIS_inf)/214;
%     recall_MSIS_inf = detection_TP_MSIS_inf / (detection_TP_MSIS_inf + detection_FN_MSIS_inf);
%     precision_MSIS_inf = detection_TP_MSIS_inf / (detection_TP_MSIS_inf + detection_FP_MSIS_inf);
%     F1_MSIS_inf = 2 * precision_MSIS_inf * recall_MSIS_inf /(precision_MSIS_inf + recall_MSIS_inf);
    
    %% 9. Plot
    set(groot, 'CurrentFigure', path_figure);   
    draw_veh_pose(veh_pose, veh_params);    	
    set(groot, 'CurrentFigure', steer_figure);
    plot(log.time(i), log.steer_cmd(i), 'b.', 'markersize', 20);
    set(groot, 'CurrentFigure', acc_figure);
    plot(log.time(i), acc, 'b.', 'markersize', 20);
    set(groot, 'CurrentFigure', lateral_error);
    plot(log.time(i), matrix_state_new(1,1), 'b.', 'markersize', 20);
    set(groot, 'CurrentFigure', heading_error);
    plot(log.time(i), matrix_state_new(3,1), 'b.', 'markersize', 20);
    set(groot, 'CurrentFigure', station_error);
    plot(log.time(i), matrix_state_new(5,1), 'b.', 'markersize', 20);
    set(groot, 'CurrentFigure',speed_error);
    plot(log.time(i), matrix_state_new(6,1), 'b.', 'markersize', 20);
    
    % MSIS-inf
%     set(groot, 'CurrentFigure', fault_detection_MSIS_inf_figure);
%     plot(log.time(i), fault_detection_MSIS_inf, 'r.', 'markersize', 20);
%     plot(log.time(i), test_MSIS_inf, 'b.', 'markersize', 20);
%     plot(log.time(i), threshold_MSIS_inf, 'g.','markersize', 5);    
    cycle_time          = toc;                
    log.cycle_time(i,:) = cycle_time;
    pause(0.01);
end
disp('simulation end!');

%% 10. GP training
%       d_GP = train_GP_model(log);
%       save('GP_u_20230710_delta_bias_0.2fault.mat','d_GP');
