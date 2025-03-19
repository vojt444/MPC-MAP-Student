function [estimated_pose] = estimate_pose(read_only_vars, public_vars)
%ESTIMATE_POSE Summary of this function goes here

% if any(isnan(read_only_vars.gnss_position))
%     estimated_pose = mean(public_vars.particles(1:read_only_vars.max_particles,:));
% else
    estimated_pose = public_vars.mu;
% end

end

