function angle_norm = angle_normalization(angle_origin)

if angle_origin > pi
    angle_norm = angle_origin - 2 * pi;
    
elseif angle_origin < -pi
    angle_norm = angle_origin + 2 * pi;
    
else
    angle_norm = angle_origin;
end

