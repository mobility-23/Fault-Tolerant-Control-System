function draw_veh_pose(veh_pose, veh_params)

x0    = veh_pose(1);
y0    = veh_pose(2);
theta = veh_pose(3);
hold on
plot(x0, y0, 'b.', 'markersize', veh_params.vehicle_size);
grid minor

x1 = x0 + veh_params.vehicle_length * cos(theta);
y1 = y0 + veh_params.vehicle_length * sin(theta);
draw_arrow([x0, y0], [x1, y1]);  

