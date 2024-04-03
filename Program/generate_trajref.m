function trajref = generate_trajref(trajref_params, roadmap_name)

v = 12 * ones(10,1);
dist_interval = trajref_params.dist_interval;   
traj1_dist = trajref_params.traj1_dist;         
if (~contains(roadmap_name,'wave_test'))
    r2 = trajref_params.r2;                        
    traj3_dist = trajref_params.traj3_dist;         
    r4 = trajref_params.r4;                      
    r5 = trajref_params.r5;                      
    r6 = trajref_params.r6;
    traj7_dist = trajref_params.traj7_dist;         
    r8 = trajref_params.r8;
    r9 = trajref_params.r9;
    r10 = trajref_params.r10;
end

default_r = 10000;   


X1 = (0 : dist_interval : traj1_dist)';  
n1 = length(X1);               
Y1 = zeros(n1, 1);              
T1 = zeros(n1, 1);             
R1 = ones(n1, 1) * default_r;   
C1 = 1 ./ R1;                 
S1 = X1;                        
V1 = ones(n1,1) * v(1);
traj1 = [X1, Y1, T1, R1, C1, S1, V1];   

if (~contains(roadmap_name,'wave_test'))
 
    theta_interval_2 = dist_interval / r2;  
    Theta_2 = (-pi/2 : theta_interval_2 : 0)';
    X2 = r2 * cos(Theta_2) + traj1(end, 1);
    Y2 = r2 * sin(Theta_2) + traj1(end, 2) + r2;
    n2 = length(X2);
    T2 = zeros(n2, 1);
    R2 = ones(n2, 1) * r2;
    C2 = 1 ./ R2;
    S2 = S1(end) * ones(n2, 1) + r2 * (0:theta_interval_2:pi/2)';
    V2 = ones(n2,1) * v(2);
    traj2 = [X2, Y2, T2, R2, C2, S2, V2];

    Y3 = (traj2(end, 2) : dist_interval : (traj2(end, 2) + traj3_dist))';
    n3 = length(Y3);
    X3 = ones(n3, 1) * traj2(end, 1);
    T3 = zeros(n3, 1);
    R3 = ones(n3, 1) * default_r;
    C3 = 1 ./ R3; 
    S3 = S2(end) * ones(n3, 1) + dist_interval * (0:1:n3-1)';
    V3 = ones(n3,1) * v(3);
    traj3 = [X3, Y3, T3, R3, C3, S3, V3];

    theta_interval_4 = dist_interval / r4;  
    Theta_4 = (0 : theta_interval_4 : pi/2)';
    X4 = r4 * cos(Theta_4) + traj3(end, 1) - r4;
    Y4 = r4 * sin(Theta_4) + traj3(end, 2) ;
    n4 = length(X4);
    T4 = zeros(n4, 1);
    R4 = ones(n4, 1) * -r4;
    C4 = 1 ./ R4; 
    S4 = S3(end) * ones(n4, 1) + r4 * (0:theta_interval_4:pi/2)';
    V4 = ones(n4,1) * v(4);
    traj4 = [X4, Y4, T4, R4, C4, S4, V4];

    theta_interval_5 = dist_interval / r5;  
    Theta_5 = (pi * 3/2 : -theta_interval_5 : pi)';
    X5 = r5 * cos(Theta_5) + traj4(end, 1);
    Y5 = r5 * sin(Theta_5) + traj4(end, 2) + r5;
    n5 = length(X5);
    T5 = zeros(n5, 1);
    R5 = ones(n5, 1) * r5;
    C5 = 1 ./ R5;
    S5 = S4(end) * ones(n5, 1) + r5 * (0:theta_interval_5:pi/2)';
    V5 = ones(n5,1) * v(5);
    traj5 = [X5, Y5, T5, R5, C5, S5, V5];

    theta_interval_6 = dist_interval / r6;  
    Theta_6 = (0 : theta_interval_6 : pi/2)';
    X6 = r6 * cos(Theta_6) + traj5(end, 1)- r6;
    Y6 = r6 * sin(Theta_6) + traj5(end, 2) ;
    n6 = length(X6);
    T6 = zeros(n6, 1);
    R6 = ones(n6, 1) * r6;
    C6 = 1 ./ R6;
    S6 = S5(end) * ones(n6, 1) + r6 * (0:theta_interval_6:pi/2)';
    V6 = ones(n6,1) * v(6);
    traj6 = [X6, Y6, T6, R6, C6, S6, V6];
    
    X7 = (traj6(end, 1) : - dist_interval : (traj6(end, 1) - traj7_dist))';
    n7 = length(X7);
    Y7 = ones(n7, 1) * traj6(end, 2);
    T7 = zeros(n7, 1);
    R7 = ones(n7, 1) * default_r;
    C7 = 1 ./ R7; 
    S7 = S6(end) * ones(n7, 1) + dist_interval * (0:1:n7-1)';
    V7 = ones(n7,1) * v(7);
    traj7 = [X7, Y7, T7, R7, C7, S7, V7];
    
    theta_interval_8 = dist_interval / r8; 
    Theta_8 = (pi/2 : theta_interval_8 : 3/2 * pi)';
    X8 = r8 * cos(Theta_8) + traj7(end, 1);
    Y8 = r8 * sin(Theta_8) + traj7(end, 2) - r8;
    n8 = length(X8);
    T8 = zeros(n8, 1);
    R8 = ones(n8, 1) * r8;
    C8 = 1 ./ R8;
    S8 = S7(end) * ones(n8, 1) + r8 * (0:theta_interval_8:pi)';
    V8 = ones(n8,1) * v(8);
    traj8 = [X8, Y8, T8, R8, C8, S8, V8];
    
    theta_interval_9 = dist_interval / r9;  
    Theta_9 = (pi/2 : -theta_interval_9 : - pi/2)';
    X9 = r9 * cos(Theta_9) + traj8(end, 1);
    Y9 = r9 * sin(Theta_9) + traj8(end, 2) - r9;
    n9 = length(X9);
    T9 = zeros(n9, 1);
    R9 = ones(n9, 1) * r9;
    C9 = 1 ./ R9;
    S9 = S8(end) * ones(n9, 1) + r9 * (0:theta_interval_9:pi)';
    V9 = ones(n9,1) * v(9);
    traj9 = [X9, Y9, T9, R9, C9, S9, V9];
    
    theta_interval_10 = dist_interval / r10; 
    Theta_10 = (pi/2 : theta_interval_10 : 3/2 * pi)';
    X10 = r10 * cos(Theta_10) + traj9(end, 1);
    Y10 = r10 * sin(Theta_10) + traj9(end, 2) - r10;
    n10 = length(X10);
    T10 = zeros(n10, 1);
    R10 = ones(n10, 1) * r10;
    C10 = 1 ./ R10;
    S10 = S9(end) * ones(n10, 1) + r10 * (0:theta_interval_10:pi)';
    V10 = ones(n10,1) * v(10);
    traj10 = [X10, Y10, T10, R10, C10, S10, V10];

    trajref = [traj1(1:n1-1, :); traj2(1:n2-1, :); traj3(1:n3-1, :);...
        traj4(1:n4-1, :); traj5(1:n5-1, :); traj6(1:n6-1, :);traj7(1:n7-1, :);...
        traj8(1:n8-1, :);traj9(1:n9-1, :);traj10];
else
    trajref = [traj1(1:n1-1, :)];
end

n = length(trajref);
for i = 1 : 1 : (n-1)
    point1 = trajref(i, 1:2);
    point2 = trajref(i+1, 1:2);
    heading_angle = calc_heading_angle(point1, point2);
    trajref(i, 3) = heading_angle;
end
trajref(n, 3) = trajref(n-1, 3);

figure('name', 'Path Curvature');
title('Path Curvature', 'fontsize', 15); 
hold on;
plot(trajref(:, 5), 'r');

inc_curvature = 0.02;
for i = (n-1) : -1 : 1
    c_next = trajref(i+1, 5);
    c_curr = trajref(i, 5);
    
    c_max = c_next + inc_curvature;
    c_min = c_next - inc_curvature;
    
    c_curr = max(c_curr, c_min);
    c_curr = min(c_curr, c_max);
    
    trajref(i, 5) = c_curr;
    trajref(i, 4) = 1 / c_curr;
end

figure('name', 'Path Curvature');
title('Path Curvature', 'fontsize', 15); 
hold on;
plot(trajref(:, 3), 'b');

for i = 1 : 1 : n
    trajref(i, 4) = limit_radius(trajref(i, 4), default_r);
    trajref(i, 5) = 1 / trajref(i, 4);
end

