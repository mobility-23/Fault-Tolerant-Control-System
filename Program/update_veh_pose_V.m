function veh_pose = update_veh_pose_V(matrix_state_new, trajref, run_distance)

n = length(trajref);
dist = zeros(n, 1);
for i = 1:n
    dist(i, :) = norm(trajref(i, 6) - run_distance);
end
[~, index] = min(dist); 
X_d = trajref(index, 1);
Y_d = trajref(index, 2);
Theta_d = trajref(index, 3);
R = trajref(index, 4);

e_y = matrix_state_new(1,1);
e_phi = matrix_state_new(3,1);
e_s = matrix_state_new(5,1);

dy = e_y * cos(Theta_d) + e_s * sin(Theta_d);
dx = e_s * cos(Theta_d) - e_y * sin(Theta_d);
X = X_d + dx;
Y = Y_d + dy;

Theta = Theta_d + e_phi;

veh_pose = [X, Y, Theta];
