function [public_vars] = robot_pos_check(read_only_vars, public_vars)

if read_only_vars.lidar_distances(1) < 0.3
    diff = read_only_vars.lidar_distances(2) - read_only_vars.lidar_distances(8);
    diff = (diff < 0)*2 -1;
    public_vars.motion_vector = [-0.5*diff, 0.5*diff];
    maneuver_start = read_only_vars.counter;
    v_angular = (public_vars.motion_vector(2) - public_vars.motion_vector(1)) / read_only_vars.agent_drive.interwheel_dist;
    degrees_per_sample = abs(v_angular) * read_only_vars.sampling_period * (180/pi);
    maneuver_length = ceil(90 / degrees_per_sample);
    public_vars.collision_detection = maneuver_start + maneuver_length;
end

end

