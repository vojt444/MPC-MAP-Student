function [public_vars] = init_transition_from_indoor(read_only_vars, public_vars)

public_vars.mu = mean(public_vars.particles(1:read_only_vars.max_particles,:));

end

