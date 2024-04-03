function steer_angle = limit_steer_angle(steer_angle, max_steer_angle)

steer_angle = min(steer_angle, max_steer_angle);
steer_angle = max(steer_angle, -max_steer_angle);
