function [steer_cmd, acc, state_nom, matrix_state_new,... 
          angular_v_des,...
          P_kminl, P_kminlnom,r_k_hat]                      = ALG_MPC(veh_pose, trajref,...
                                                              mpc_params, veh_params, time_step,...
                                                              matrix_state, angular_v_des,... 
                                                              P_kminl, P_kminlnom,...
                                                              log, i)


% Projection point of the vehicle state
[~, index] = calc_nearest_point(veh_pose, trajref);
ref_pose   = calc_proj_pose(veh_pose(1:2), trajref(index, 1:3),trajref(index + 1, 1:3));
delta_x    = (veh_pose - ref_pose)';

% Command

 [steer_command,acc,state_nom,matrix_state_new,... 
  angular_v_des, P_kminl, P_kminlnom,r_k_hat]                        = calc_mpc(trajref, delta_x,... 
                                                                       veh_pose, ref_pose, ...
                                                                       mpc_params, index, veh_params,...
                                                                       matrix_state, angular_v_des,...
                                                                       P_kminl, P_kminlnom,...
                                                                       log, i);
                                                     
steer_cmd =  steer_command;
steer_cmd = limit_steer_angle(steer_cmd, veh_params.max_steer_angle);
