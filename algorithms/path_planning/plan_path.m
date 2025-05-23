function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

planning_required = 1;

if planning_required
    
    for i = public_vars.max_wall_width:-1:1
        public_vars.walls_width = i;
        path = astar(read_only_vars, public_vars);
        if ~isempty(path)
            x_path_norm = path(:,1)/read_only_vars.discrete_map.dims(1)*read_only_vars.map.limits(3) + read_only_vars.map.limits(1);
            y_path_norm = path(:,2)/read_only_vars.discrete_map.dims(2)*read_only_vars.map.limits(4) + read_only_vars.map.limits(2);
            path = smooth_path([y_path_norm  x_path_norm]);
            % path = [y_path_norm  x_path_norm];
            break;
        end
    end
    
else
    % line01 = [1,1;1,2];
    % line02 = [1,2;1,3];
    % line03 = [1,3;1,4];
    % line04 = [1,4;1,5];
    % line05 = [1,5;1,6];
    % line06 = [1,6;1,7];
    % line07 = [1,7;1,7.5];

    line0 = [1,1;1,7.5];
    arc0 = generate_arc_waypoints(2,7.5,1,90,90,'CW',10);
    line1 = [2,8.5;3.5,8.5];
    arc1 = generate_arc_waypoints(3.5, 7, 1.5, 0, 90, 'CW', 15);
    line2 = [5,7;5,4];
    arc2 = generate_arc_waypoints(7, 4, 2, 180, 180, 'CCW', 20);
    line3 = [9,4;9,9];
    % public_vars.path = [line01; line02; line03; line04; line05; line06; line07; arc0; line1; arc1; line2; arc2; line3];
    path = [line0; arc0; line1; arc1; line2; arc2; line3];

    % arc0 = generate_arc_waypoints(9,2,7,0,180,'CW',50);
    % path = [arc0];

end

end

