function log = log_init(time_step, simulation_stop_time, x0, zhat0, u0)

log_size               = simulation_stop_time / time_step;    
log.time               = zeros(log_size, 1);                  
log.steer_cmd          = zeros(log_size, 1);                
log.acc                = zeros(log_size, 1);                  
log.velocity           = zeros(log_size, 1);
log.veh_pose           = zeros(log_size, 3);                 
log.dist               = zeros(log_size, 1);                
log.delta_x            = zeros(log_size, 3);                
log.solver_run_time    = zeros(log_size, 1);               
log.angular_v_des_vec  = zeros(log_size, 1);

log.xhat               = zeros(6, log_size);             
log.xnom               = zeros(6, log_size);             
log.u                  = zeros(2, log_size);            

log.zhat               = zeros(3, log_size);            
log.xhat(:,1)          = x0;
log.xnom(:,1)          = x0;
log.velocity(:,1)      = 10;


%% Detector
log.r_k_hat            = zeros(6, log_size);                 
log.MSIS_inf           = zeros(log_size,1);
log.fault_detection_MSIS_inf = zeros(log_size,1);



