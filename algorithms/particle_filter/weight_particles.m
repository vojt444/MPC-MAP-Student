function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

valid_dist = ~isinf(lidar_distances);

diff = (particle_measurements(:, valid_dist) - lidar_distances(valid_dist)).^2;

weights = 1./(sqrt(sum(diff,2)));
weights = weights/sum(weights);

end
