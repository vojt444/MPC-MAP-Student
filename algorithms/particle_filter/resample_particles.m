function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here

new_particles = zeros(size(particles));
num_particles = length(particles);

index = randi(num_particles);
for i = 1:num_particles
    beta = rand()*2*max(weights);
    while weights(index) < beta
        beta = beta - weights(index);
        index = index + 1;
        if index > num_particles
            index = 1;
        end
    end
    new_particles(i,:) = particles(index,:);
end

end
