function veh_params = load_veh_params

veh_params.mass = 2050;
veh_params.Iz = 3344;
veh_params.Calfa = 80000;
veh_params.Calfa_front = veh_params.Calfa;
veh_params.Calfa_rear = veh_params.Calfa;
veh_params.lf = 1.43;
veh_params.lr = 1.47;
veh_params.velocity = 10;

veh_params.wheel_base       = 2.9;                            
veh_params.max_steer_angle  = 53 / 180 * pi;                        
veh_params.max_angular_vel  = 53 / 180 * pi;                        
veh_params.Vx               = veh_params.velocity;

veh_params.v_des            = 15;
veh_params.angular_v        = 0;
veh_params.vehicle_size     = 20;                                  
veh_params.vehicle_length   = veh_params.velocity *  0.08;
veh_params.max_acceleration = 3;
veh_params.max_deceleration = 3;
end