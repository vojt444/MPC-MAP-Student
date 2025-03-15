function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

rand_pos = randn(length(read_only_vars.gnss_history),1);
rand_pos = rand_pos/max(rand_pos)*2*pi;

public_vars.kf.C = [1 0 0; 0 1 0];
public_vars.kf.R = [0.001 0 0; 0 0.002 0; 0 0 0.002];
public_vars.kf.Q = cov(read_only_vars.gnss_history);

public_vars.mu = [mean(read_only_vars.gnss_history) mean(rand_pos)];
public_vars.sigma = cov([read_only_vars.gnss_history rand_pos]);

public_vars.kf.C = [];
public_vars.kf.R = [];
public_vars.kf.Q = [];

public_vars.mu = [];
public_vars.sigma = [];

end

