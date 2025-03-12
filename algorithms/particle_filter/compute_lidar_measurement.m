function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));
for i = 1:length(lidar_config)
    intersections = ray_cast([pose(1) pose(2)], map.walls, pose(3) + lidar_config(i));
    NaN_rows = any(isnan(intersections),2);
    intersections(NaN_rows,:) = [];
    if isempty(intersections)
        measurement(i) = inf;
    else
        measurement(i) = min(sqrt((intersections(:,1) - pose(1)).^2 + (intersections(:,2) - pose(2)).^2));
    end
end

end

