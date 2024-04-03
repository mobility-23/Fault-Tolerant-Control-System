function radius = limit_radius(radius, max_radius)

radius = min(radius, max_radius);
radius = max(radius, -max_radius);