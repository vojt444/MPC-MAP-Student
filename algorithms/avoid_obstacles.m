function [public_vars] = avoid_obstacles(read_only_vars, public_vars)
    % Initialize parameters for smooth avoidance
    if ~isfield(public_vars, 'prev_motion')
        public_vars.prev_motion = [0, 0];
    end
    
    % Parameters
    safety_dist = 0.2; % Detection radius
    repulsion_gain = 0.5; 
    smoothing_factor = 0.4; % Motion smoothing (0-1)
    
    % Original motion vector
    orig_motion = public_vars.motion_vector;
    
    % Extract LiDAR information
    lidar_dist = read_only_vars.lidar_distances;
    lidar_angles = read_only_vars.lidar_config;
    
    % Calculate repulsive forces from obstacles
    fx = 0; 
    fy = 0;
    
    for i = 1:length(lidar_dist)
        dist = lidar_dist(i);
        
        % Only consider valid measurements within safety distance
        if ~isinf(dist) && dist < safety_dist && dist > 0
            % Calculate repulsion magnitude (stronger when closer)
            magnitude = repulsion_gain * (1 - dist/safety_dist)^2;
            
            % Calculate repulsion direction (away from obstacle)
            angle = lidar_angles(i) + public_vars.estimated_pose(3);
            fx = fx - magnitude * cos(angle);
            fy = fy - magnitude * sin(angle);
        end
    end
    
    % Convert forces to wheel velocities
    force_magnitude = sqrt(fx^2 + fy^2);
    
    if force_magnitude > 0.01
        % Calculate force direction
        force_angle = atan2(fy, fx);
        
        % Calculate angle difference to robot heading
        robot_heading = public_vars.estimated_pose(3);
        angle_diff = force_angle - robot_heading;
        
        % Normalize angle to [-pi, pi]
        angle_diff = mod(angle_diff + pi, 2*pi) - pi;
        
        % Calculate wheel velocities for avoidance
        base_speed = 0.4;
        turn_rate = 3 * sin(angle_diff);
        
        v_R = base_speed + turn_rate * read_only_vars.agent_drive.interwheel_dist/2;
        v_L = base_speed - turn_rate * read_only_vars.agent_drive.interwheel_dist/2;
        
        % Scale forces based on magnitude
        influence = min(1.0, force_magnitude);
        
        % Blend original motion with avoidance motion
        avoidance_motion = [v_R, v_L];
        blended_motion = (1-influence) * orig_motion + influence * avoidance_motion;
        
        % Apply temporal smoothing
        smoothed_motion = smoothing_factor * public_vars.prev_motion + (1-smoothing_factor) * blended_motion;
        
        % Apply motion limits
        max_vel = read_only_vars.agent_drive.max_vel;
        smoothed_motion(1) = max(-max_vel, min(max_vel, smoothed_motion(1)));
        smoothed_motion(2) = max(-max_vel, min(max_vel, smoothed_motion(2)));
        
        % Update motion vector
        public_vars.motion_vector = smoothed_motion;
        public_vars.prev_motion = smoothed_motion;
    else
        % No obstacles detected, smooth transition to original motion
        smoothed_motion = smoothing_factor * public_vars.prev_motion + (1-smoothing_factor) * orig_motion;
        
        public_vars.motion_vector = smoothed_motion;
        public_vars.prev_motion = smoothed_motion;
    end
end