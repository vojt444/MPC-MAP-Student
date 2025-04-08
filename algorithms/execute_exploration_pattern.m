function [public_vars] = execute_exploration_pattern(read_only_vars, public_vars)
    % Different movement patterns to gather maximum information
    
    % Pattern phase progresses over time
    pattern_phase = mod(read_only_vars.counter, 20);
    
    if pattern_phase < 5
        % Rotate in place (first pattern)
        rotation_speed = 0.5;
        public_vars.motion_vector = [rotation_speed, -rotation_speed];
        
    elseif pattern_phase < 10
        % Move forward
        forward_speed = 0.3;
        public_vars.motion_vector = [forward_speed, forward_speed];
        
    elseif pattern_phase < 15
        % Rotate the other way
        rotation_speed = 0.5;
        public_vars.motion_vector = [-rotation_speed, rotation_speed];
        
    else
        % Short forward movement
        forward_speed = 0.3;
        public_vars.motion_vector = [forward_speed, forward_speed];
    end
    
    % Always apply obstacle avoidance during exploration
    public_vars = avoid_obstacles(read_only_vars, public_vars);
    
    % Increment pattern step
    public_vars.exploration_pattern = public_vars.exploration_pattern + 1;
end