function heading_angle = calc_heading_angle(point1, point2)

delta_y = point2(2) - point1(2);
delta_x = point2(1) - point1(1);

heading_angle = mod(atan2(delta_y, delta_x), 2*pi);

