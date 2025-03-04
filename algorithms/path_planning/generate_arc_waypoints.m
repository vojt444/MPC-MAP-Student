function waypoints = generate_arc_waypoints(cx, cy, radius, start_angle_deg, angle_deg, dir, N)

start_angle_rad = deg2rad(start_angle_deg);
end_angle_rad = deg2rad(start_angle_deg + angle_deg);

% theta = linspace(0, deg2rad(angle_deg), N); 
theta = linspace(start_angle_rad, end_angle_rad, N); 

x = cx + radius * cos(theta);
y = cy + radius * sin(theta);

waypoints = zeros(length(x),2);
for i=1:length(x)
    waypoints(i,1) = x(i); 
end

for i=1:length(y)
    waypoints(i,2) = y(i); 
end

if strcmp(dir,'CW')
    waypoints = flip(waypoints);
elseif strcmp(dir, 'CCW')

else
    error("Use only 'CW' or 'CCW' direction parameter.");
end
    

end