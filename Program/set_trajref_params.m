function [trajref_params, simulation_stop_y, simulation_stop_time] =...
    set_trajref_params(roadmap_name, veh_params)

trajref_params.dist_interval      = 0.2;                        

switch (roadmap_name)
    
    case 'small_circle'
        trajref_params.traj1_dist = 30;                   
        trajref_params.r2         = 5;                        
        trajref_params.traj3_dist = 15;                     
        trajref_params.r4         = 5;                       
        trajref_params.r5         = 5;                        
        trajref_params.traj6_dist = 30;                        
        simulation_stop_y         = 30;                         
        simulation_stop_time      = 90 / veh_params.velocity;   
   
    case 'wave_test'
        trajref_params.traj1_dist = 230;
        
        simulation_stop_y         = 60;
        simulation_stop_time      = 180 / veh_params.velocity;
        
    case 'big_circle'        
        trajref_params.traj1_dist = 60;
        trajref_params.r2         = 10;
        trajref_params.traj3_dist = 10;
        trajref_params.r4         = 10;
        trajref_params.r5         = 20;
        trajref_params.r6         = 5;
        trajref_params.traj7_dist = 35;
        trajref_params.r8         = 7.5;
        trajref_params.r9         = 12.5;
        trajref_params.r10        = 7.5;
        
        simulation_stop_y         = 200;
        simulation_stop_time      = 230 / veh_params.velocity;

        
    otherwise
        disp('The roadmap does not exit!');
end

