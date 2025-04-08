function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

init_theta = 0;

public_vars.kf.C = [1 0 0; 
                    0 1 0];
public_vars.kf.R = [0.0005 0 0; 
                    0 0.0005 0; 
                    0 0 0.00009];

if any(isnan(read_only_vars.gnss_history(:,:)))
    public_vars.kf.Q = [0.2308,	-0.0237; -0.0237, 0.3647];
    public_vars.mu = [0, 0, 0];
else
    public_vars.kf.Q = cov(read_only_vars.gnss_history(:,:));
    public_vars.mu = [mean(read_only_vars.gnss_history,1) init_theta];
end

public_vars.sigma = diag([public_vars.kf.Q(1,1), public_vars.kf.Q(2,2), 10]);



end

