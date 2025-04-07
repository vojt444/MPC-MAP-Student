function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

[distance_vector, public_vars] = get_target(public_vars.estimated_pose, public_vars);

% II. Compute motion vector
angle_error = atan2(distance_vector(2),distance_vector(1)) - public_vars.estimated_pose(3);

v = 0.5;
omega = 2*v*sin(angle_error)/read_only_vars.agent_drive.interwheel_dist;

v_L = v - (omega*read_only_vars.agent_drive.interwheel_dist/2);
v_R = v + (omega*read_only_vars.agent_drive.interwheel_dist/2);

v_L = max(-2, min(2,v_L));
v_R = max(-2, min(2,v_R));

public_vars.motion_vector = [v_R v_L];

% public_vars.motion_vector = [0 0];

end