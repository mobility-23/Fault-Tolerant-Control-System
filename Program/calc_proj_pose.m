function proj_pose = calc_proj_pose(p0, p1, p2)

tol = 0.0001;
proj_pose = [0, 0, 0];

if abs(p2(1) - p1(1)) < tol
    
    x = p1(1);     
    y = p0(2);     

elseif abs(p2(2) - p1(2)) < tol
 
    x = p0(1);     
    y = p1(2);  
    
else
    k1 = (p2(2) - p1(2)) / (p2(1) - p1(1)); 
    k2 = -1 / k1;  
    
    x = (p0(2) - p1(2) + k1 * p1(1) - k2 * p0(1)) / (k1 - k2);
    y = p0(2) + k2 * (x - p0(1));
end

proj_pose(1) = x;
proj_pose(2) = y;

dist = norm(p2(1:2) - p1(1:2));       
dist2 = norm(p2(1:2) - proj_pose(1:2)); 

ratio = dist2 / dist;
theta = ratio * p1(3) + (1 - ratio) * p2(3);

proj_pose(3) = theta;


