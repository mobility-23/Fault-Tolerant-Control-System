function con_params = load_con_params(veh_params)

   con_params.u_ll       = [-veh_params.max_steer_angle; -3];
   con_params.u_uu       = [veh_params.max_steer_angle; 3];
   con_params.delta_u_ll = [-veh_params.max_steer_angle/5; -0.5];
   con_params.delta_u_uu = [veh_params.max_steer_angle/5; 0.5];
   con_params.x_ll       = [-2.5; -veh_params.max_steer_angle];
   con_params.x_uu       = [2.5; veh_params.max_steer_angle];
   con_params.g_x = [1 0 0 0 0 0; 0 0 1 0 0 0];
   con_params.g_u = diag(1,1);
   con_params.g_delta_u = 1;
   
end