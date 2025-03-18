function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

rand_theta = randn(length(read_only_vars.gnss_history),1);
rand_theta = rand_theta/max(rand_theta)*2*pi;

public_vars.kf.C = [1 0 0; 0 1 0];
public_vars.kf.R = [0.001 0 0; 
                    0 0.001 0; 
                    0 0 0.002];
public_vars.kf.Q = cov(read_only_vars.gnss_history);

public_vars.mu = [mean(read_only_vars.gnss_history,1) mean(rand_theta)];
public_vars.sigma = cov([read_only_vars.gnss_history rand_theta']);

if any(isnan(read_only_vars.gnss_history(:,1)))
    public_vars.Q = [0.2389 -0.0147;
                     -0.0147 0.2087];

    public_vars.sigma = [0.0101080467194839	-0.00148707538536078	-0.00167050923297438;
                    -0.00148707538536077	0.0116692796614171	0.00152599789264892;
                    -0.00167050923297438	0.00152599789264892	0.00347058150179685];
end

end

