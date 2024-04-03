function draw_arrow(start_point, end_point)

K = 0.25;      
linewidth = 2; 

theta     = pi / 8; 
A1        = [cos(theta), -sin(theta);
            sin(theta), cos(theta)];  
theta     = -theta;
A2        = [cos(theta), -sin(theta);
            sin(theta), cos(theta)];  

arrow     = start_point' - end_point'; 
arrow_1   = A1 * arrow;
arrow_2   = A2 * arrow;
arrow_1   = K * arrow_1 + end_point';
arrow_2   = K * arrow_2 + end_point';

hold on
plot([start_point(1), end_point(1)], [start_point(2), end_point(2)],...
    'k', 'linewidth', linewidth);
plot([arrow_1(1), end_point(1)], [arrow_1(2), end_point(2)], 'k',...
    'linewidth', linewidth);
plot([arrow_2(1), end_point(1)], [arrow_2(2), end_point(2)], 'k',...
    'linewidth', linewidth);
grid minor
axis equal

