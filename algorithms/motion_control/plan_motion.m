function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector
if read_only_vars.counter < 100
    public_vars.motion_vector = [0.5, 0.5];
elseif read_only_vars.counter >= 100 && read_only_vars.counter < 225
    public_vars.motion_vector = [0.5, 0.55];
elseif read_only_vars.counter >= 225 && read_only_vars.counter < 300
    public_vars.motion_vector = [0.5, 0.5];
elseif read_only_vars.counter >= 300 && read_only_vars.counter < 380
    public_vars.motion_vector = [0.57, 0.5];
else
    public_vars.motion_vector = [1, 1];
end

end