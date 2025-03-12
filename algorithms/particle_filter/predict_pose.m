function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

x = old_pose(1);
y = old_pose(2);
theta = old_pose(3);

v_R = motion_vector(1);
v_L = motion_vector(2);

v = (v_R + v_L)/2;
omega = (v_R - v_L)/read_only_vars.agent_drive.interwheel_dist;

v_noisy = v + randn;
omega_noisy = omega + randn;

x_new = x + v_noisy*read_only_vars.sampling_period*cos(theta);
y_new = y + v_noisy*read_only_vars.sampling_period*sin(theta);
theta_new = theta + omega_noisy*read_only_vars.sampling_period;
theta_new = mod(theta_new + pi, 2*pi) - pi;

new_pose = [x_new y_new theta_new];

end

