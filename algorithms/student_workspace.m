function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars.target_index = 1;
    public_vars.delay_const = 50;
    public_vars.max_wall_width = 5;
    public_vars.walls_width = 0;
end


% 9. Update particle filter
if any(isnan(read_only_vars.gnss_position))
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
end

% 10. Update Kalman filter
if read_only_vars.counter == public_vars.delay_const
    public_vars = init_kalman_filter(read_only_vars,public_vars);
end
if read_only_vars.counter > public_vars.delay_const
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
end

% 11. Estimate current robot position
if read_only_vars.counter >= public_vars.delay_const
    public_vars.estimated_pose = estimate_pose(read_only_vars, public_vars); % (x,y,theta)
end

% 12. Path planning
if read_only_vars.counter == public_vars.delay_const 
    public_vars.path = plan_path(read_only_vars, public_vars);
end

% 13. Plan next motion command
if read_only_vars.counter > public_vars.delay_const
    public_vars = plan_motion(read_only_vars, public_vars);
end


% % stats measuring
% sensor = 2; %1 for LiDAR, 2 for GNSS statistics, other for nothing
% periods = 100; %number of measurements
% 
% % LiDAR stats
% if sensor == 1
%     if read_only_vars.counter <= periods
%         for channel = 1:8
%             public_vars.data_lidar(channel, read_only_vars.counter) = read_only_vars.lidar_distances(channel);
%         end
%     end
%     if read_only_vars.counter == periods + 1
%         mean_lidar = 0;
%         variance_lidar = zeros(8,1);
%         public_vars.std_lidar = zeros(8,1);
%         for channel = 1:8
%             mean_lidar = sum(public_vars.data_lidar(channel,:))/length(public_vars.data_lidar(channel,:));
%             public_vars.std_lidar(channel) = std(public_vars.data_lidar(channel,:));
% 
%             figure(channel + 2);
%             histogram(public_vars.data_lidar(channel,:));
%             title(['Channel ', num2str(channel), ' LiDAR data histogram']);
% 
%             public_vars.cov_lidar = cov(public_vars.data_lidar');
% 
%             if channel == 1
%                 x_lidar = (-5*public_vars.std_lidar(channel):0.01:5*public_vars.std_lidar(channel));
%                 PDF_lidar = norm_pdf(x_lidar, 0, public_vars.std_lidar(channel));
%                 figure(2)
%                 plot(x_lidar, PDF_lidar);
%                 grid on;
%                 title('LiDAR sensor PDF');
%             end
%         end
%     end
% end
% 
% % GNSS stats
% if sensor == 2
%     if read_only_vars.counter <= periods
%         for axe = 1:2
%             public_vars.data_gnss(axe, read_only_vars.counter) = read_only_vars.gnss_position(axe);
%         end
%     end
%     if read_only_vars.counter == periods + 1
%         mean_gnss = 0;
%         variance_gnss = zeros(2,1);
%         public_vars.std_gnss = zeros(2,1);
%         for axe = 1:2
%             mean_gnss = sum(public_vars.data_gnss(axe,:))/length(public_vars.data_gnss(axe,:));
%             public_vars.std_gnss(axe) = std(public_vars.data_gnss(axe,:));
% 
%             figure(axe + 2)
%             histogram(public_vars.data_gnss(axe,:))
%             title(['Axe ', num2str(axe), ' GNSS data histogram']);
% 
%             public_vars.cov_gnss = cov(public_vars.data_gnss');
% 
%             if axe == 1
%                 x_gnss = (-5*public_vars.std_gnss(axe):0.01:5*public_vars.std_gnss(axe));
%                 PDF_gnss = norm_pdf(x_gnss, 0, public_vars.std_gnss(axe));
%                 figure(2);
%                 plot(x_gnss, PDF_gnss);
%                 grid on;
%                 title('GNSS sensor PDF');
%             end
%         end
%     end
% end


end

