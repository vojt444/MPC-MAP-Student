function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

diff = (particle_measurements - lidar_distances).^2;

weights = 1./(sqrt(sum(diff,2)));
weights = weights/sum(weights);

end

