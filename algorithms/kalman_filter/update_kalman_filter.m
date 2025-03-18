function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;

v_L = public_vars.motion_vector(2);
v_R = public_vars.motion_vector(1);

v = (v_L + v_R)/2;
w = (v_R - v_L)/read_only_vars.agent_drive.interwheel_dist;

% I. Prediction
u = [v w];
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement
z = read_only_vars.gnss_position';
[mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end

