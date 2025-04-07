function [public_vars] = init_transition_from_outdoor(read_only_vars, public_vars)

public_vars.particles = zeros(read_only_vars.max_particles, 3);
public_vars.particles = public_vars.particles + public_vars.mu;

end

