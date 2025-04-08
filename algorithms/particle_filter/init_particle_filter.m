function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here

max_x = max(read_only_vars.map.limits(3) - read_only_vars.map.limits(1));
max_y = max(read_only_vars.map.limits(4) - read_only_vars.map.limits(2));

part_x = zeros(1, read_only_vars.max_particles);
part_y = zeros(1, read_only_vars.max_particles);
part_theta = zeros(1, read_only_vars.max_particles);

for i = 1:read_only_vars.max_particles
    part_x(i) = rand*max_x;
    part_y(i) = rand*max_y;
    part_theta(i) = rand*2*pi;
end

public_vars.particles = [part_x' part_y' part_theta'];

end
