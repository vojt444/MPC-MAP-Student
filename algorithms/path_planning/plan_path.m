function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

planning_required = 0;

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    line1 = [2,8.5;3.5,8.5];
    arc1 = generate_arc_waypoints(3.5, 7, 1.5, 0, 90, 'CW', 15);
    line2 = [5,7;5,4];
    arc2 = generate_arc_waypoints(7, 4, 2, 180, 180, 'CCW', 20);
    line3 = [9,4;9,9];
    public_vars.path = [line1; arc1; line2; arc2; line3];
    path = public_vars.path;

end

end

