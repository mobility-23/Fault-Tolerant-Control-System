function draw_traj_curvature(trajref)

k = 10;               
n = length(trajref);   

for i = 1 : 1 : n
    x            = trajref(i, 1);                        
    y            = trajref(i, 2);                       
    theta        = trajref(i, 3);                         
    r            = trajref(i, 4);                       
    c            = trajref(i, 5);                       
    circle_theta = theta - pi / 2;                    
    xo           = x - r * cos(circle_theta);            
    yo           = y - r * sin(circle_theta);           
    
    xc           = xo + (r + k * c) * cos(circle_theta); 
    yc           = yo + (r + k * c) * sin(circle_theta); 
    
    plot([x, xc], [y, yc], 'b');                         
    hold on;
end


