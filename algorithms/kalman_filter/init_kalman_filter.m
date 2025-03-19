function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

init_theta = 0;

public_vars.kf.C = [1 0 0; 
                    0 1 0];
public_vars.kf.R = [0.0002 0 0; 
                    0 0.0002 0; 
                    0 0 0.0002];
public_vars.kf.Q = [0.2389 -0.0147;
                    -0.0147 0.2087];

public_vars.mu = [mean(read_only_vars.gnss_history,1) init_theta];
public_vars.sigma = cov([mean(read_only_vars.gnss_history,1) init_theta]);

end

