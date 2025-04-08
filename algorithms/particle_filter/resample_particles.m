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

    % new_particles = zeros(size(particles));
    % num_particles = size(particles, 1);
    % 
    % % Calculate effective sample size
    % n_eff = 1/sum(weights.^2);
    % 
    % % Only resample if effective sample size is below threshold
    % if n_eff < num_particles/2
    %     % Typical resampling
    %     index = randi(num_particles);
    %     for i = 1:num_particles
    %         beta = rand()*2*max(weights);
    %         while weights(index) < beta
    %             beta = beta - weights(index);
    %             index = mod(index, num_particles) + 1;
    %         end
    %         new_particles(i,:) = particles(index,:);
    % 
    %         % Add small random noise to maintain diversity
    %         new_particles(i,1) = new_particles(i,1) + randn*0.02;
    %         new_particles(i,2) = new_particles(i,2) + randn*0.02;
    %         new_particles(i,3) = new_particles(i,3) + randn*0.05;
    %     end
    % else
    %     % Keep original particles if diversity is good
    %     new_particles = particles;
    % end

end
