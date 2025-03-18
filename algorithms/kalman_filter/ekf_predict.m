function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

v = u(1);
w = u(2);
theta = mu(3);

dx = v*cos(theta)*sampling_period;
dy = v*sin(theta)*sampling_period;
dtheta = w*sampling_period;

new_mu = mu + [dx dy dtheta];

G = [1 0 -sin(theta)*v*sampling_period;
     0 1 cos(theta)*v*sampling_period;
     0 0 1];

new_sigma = G*sigma*G' + kf.R;

end

