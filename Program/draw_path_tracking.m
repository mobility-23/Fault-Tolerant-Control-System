function [path_figure, steer_figure,... 
          acc_figure, lateral_error,...
          heading_error, station_error,...
          speed_error,...
          fault_detection_MSIS_inf_figure]    = draw_path_tracking(path_tracking_alg, roadmap_name,... 
                                                                 trajref, veh_pose, command,...
                                                                 veh_params, simulation_time,... 
                                                                 simulation_stop_time, matrix_state,... 
                                                                 matrix_control,...
                                                                 fault_detection_MSIS_inf)

delta_x0 = veh_pose - trajref(1,1:3);


path_figure            = figure('name', 'Path Tracking');
hold on;
grid minor;
axis equal; 
path_figure_title_name = set_title_name(path_tracking_alg);
title(path_figure_title_name, 'fontsize', 15);  
path_figure_ylimit     = set_y_limits(roadmap_name);
ylim(path_figure_ylimit);              
xlabel('X(m)', 'fontsize', 15);         
ylabel('Y(m)', 'fontsize', 15);       
plot(trajref(:, 1), trajref(:, 2), 'r.', 'markersize', 20); 
draw_traj_curvature(trajref); 
draw_veh_pose(veh_pose, veh_params);    


steer_figure = figure('name', 'Path Tracking');
hold on;
grid minor;
steer_figure_title_name = set_title_name(path_tracking_alg);
title(steer_figure_title_name, 'fontsize', 15);
steer_figure_xlimit = simulation_stop_time;
lateral_error_figure_ylimit = veh_params.max_steer_angle / pi * 180;
axis([0, steer_figure_xlimit, -lateral_error_figure_ylimit, lateral_error_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('steer command(deg)', 'fontsize', 15);
plot(simulation_time, matrix_control(1), 'b.', 'markersize', 20);
 

acc_figure = figure('name', 'Path Tracking');
hold on;
grid minor;
acc_figure_title_name = set_title_name(path_tracking_alg);
title(acc_figure_title_name, 'fontsize', 15);
acc_figure_xlimit = simulation_stop_time;
lateral_error_figure_ylimit = veh_params.max_acceleration + 1;
axis([0, acc_figure_xlimit, -lateral_error_figure_ylimit, lateral_error_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('acc command(m/s^2)', 'fontsize', 15);
plot(simulation_time, matrix_control(2), 'b.', 'markersize', 20);


lateral_error            = figure('name', 'Path Tracking');
hold on;
grid minor;
lateral_error_title_name = set_title_name(path_tracking_alg);
title(lateral_error_title_name, 'fontsize', 15);  
lateral_error_figure_xlimit = simulation_stop_time;
lateral_error_figure_ylimit = 4;
axis([0, lateral_error_figure_xlimit, -lateral_error_figure_ylimit, lateral_error_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('lateral error(m)', 'fontsize', 15);
plot(simulation_time, matrix_state(1), 'b.', 'markersize', 20);


heading_error            = figure('name', 'Path Tracking');
hold on;
grid minor;
heading_error_title_name = set_title_name(path_tracking_alg);
title(heading_error_title_name, 'fontsize', 15);  
heading_error_figure_xlimit = simulation_stop_time;
heading_error_figure_ylimit = veh_params.max_steer_angle;
axis([0, heading_error_figure_xlimit, -heading_error_figure_ylimit, heading_error_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('heading error(deg)', 'fontsize', 15);
plot(simulation_time, matrix_state(3), 'b.', 'markersize', 20);


station_error            = figure('name', 'Path Tracking');
hold on;
grid minor;
station_error_title_name = set_title_name(path_tracking_alg);
title(station_error_title_name, 'fontsize', 15);  
station_error_figure_xlimit = simulation_stop_time;
station_error_figure_ylimit = 2;
axis([0, station_error_figure_xlimit, -station_error_figure_ylimit, station_error_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('station error(m)', 'fontsize', 15);
plot(simulation_time, matrix_state(5), 'b.', 'markersize', 20);


speed_error            = figure('name', 'Path Tracking');
hold on;
grid minor;
speed_error_title_name = set_title_name(path_tracking_alg);
title(speed_error_title_name, 'fontsize', 15);  
speed_error_figure_xlimit = simulation_stop_time;
speed_error_figure_ylimit = 8;
axis([0, speed_error_figure_xlimit, -speed_error_figure_ylimit, speed_error_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('speed error(m/s)', 'fontsize', 15);
plot(simulation_time, matrix_state(6), 'b.', 'markersize', 20);


fault_detection_MSIS_inf_figure   = figure('name','Fault detection MSIS_inf');
hold on;
grid minor;
fault_detection_MSIS_inf_title_name = set_title_name(path_tracking_alg);
title(fault_detection_MSIS_inf_title_name, 'fontsize', 15);  
fault_detection_MSIS_inf_figure_xlimit = simulation_stop_time;
fault_detection_MSIS_inf_figure_ylimit = 20;
axis([0, fault_detection_MSIS_inf_figure_xlimit, -0.5, fault_detection_MSIS_inf_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('MSISinf fault detection', 'fontsize', 15);
plot(simulation_time, fault_detection_MSIS_inf, 'r.', 'markersize', 20);





