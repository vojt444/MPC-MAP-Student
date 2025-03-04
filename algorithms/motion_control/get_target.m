function [public_vars, target] = get_target(estimated_pose, public_vars)
%GET_TARGET Summary of this function goes here
max_error = 0.15;

target_point = public_vars.path(public_vars.target_index,:);
distance_vector = target_point - estimated_pose(1,1:2);

error = sqrt(distance_vector(1)^2 + distance_vector(2)^2);

if error <= max_error
    if public_vars.target_index + 1 > length(public_vars.path)
        public_vars.motion_vector = [0,0];
        return;
    end
    public_vars.target_index = public_vars.target_index + 1;
end
target = public_vars.path(public_vars.target_index,:);

end % end function



