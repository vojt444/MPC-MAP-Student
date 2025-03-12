function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

[distance_vector, public_vars] = get_target(public_vars.estimated_pose, public_vars);

Kp = 1.3; % Proportional gain
Ki = 0.8; % Integral gain
Kd = 0.5; % Derivative gain

% II. Compute motion vector
theta = atan2(distance_vector(2),distance_vector(1));

angle_error = theta - public_vars.estimated_pose(3);
% angle_error = mod(angle_error + pi, 2*pi) - pi;

P_term = Kp * angle_error;
I_term = Ki * read_only_vars.sampling_period * public_vars.int_sum;
D_term = Kd * (angle_error - public_vars.prev_error);

omega = P_term + I_term + D_term;
public_vars.prev_error = angle_error;
public_vars.int_sum = public_vars.int_sum + angle_error;

v = distance_vector(1)*cos(theta) + distance_vector(2)*sin(theta);
omega = max(-pi/4, min(pi/4,omega));

v_L = (2*v - omega*read_only_vars.agent_drive.interwheel_dist)/2;
v_R = (2*v + omega*read_only_vars.agent_drive.interwheel_dist)/2;

v_L = max(-2, min(2,v_L));
v_R = max(-2, min(2,v_R));

public_vars.motion_vector = [v_R v_L];

% public_vars.motion_vector = [0 0];

end