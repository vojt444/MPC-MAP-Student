function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

[public_vars, target] = get_target(public_vars.estimated_pose, public_vars);

% II. Compute motion vector

distance_vect = target(1,:) - public_vars.estimated_pose(1:2);

theta = atan2(distance_vect(2),distance_vect(1));

angle_error = theta - public_vars.estimated_pose(3);
angle_error = mod(angle_error + pi, 2*pi) - pi;

v = distance_vect(1)*cos(theta) + distance_vect(2)*sin(theta);
omega = max(-pi/4, min(pi/4,angle_error));

v_L = (2*v - omega*read_only_vars.agent_drive.interwheel_dist)/2;
v_R = (2*v + omega*read_only_vars.agent_drive.interwheel_dist)/2;

public_vars.motion_vector = [v_R, v_L];


end