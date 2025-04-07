function [path] = astar(read_only_vars, public_vars)

extended_map = extend_map(read_only_vars.discrete_map.map, public_vars.walls_width);

[rows, cols] = size(extended_map);

robot_est_pos = [public_vars.estimated_pose(1) public_vars.estimated_pose(2)];
robot_disc_row = round(((robot_est_pos(1) - read_only_vars.map.limits(1))/(read_only_vars.map.limits(3) - read_only_vars.map.limits(1)))*read_only_vars.discrete_map.dims(1));
robot_disc_col = round(((robot_est_pos(2) - read_only_vars.map.limits(2))/(read_only_vars.map.limits(4) - read_only_vars.map.limits(2)))*read_only_vars.discrete_map.dims(2));

start = [robot_disc_col robot_disc_row];
goal = [read_only_vars.discrete_map.goal(2) read_only_vars.discrete_map.goal(1)];

if extended_map(start(1), start(2)) == 1 || extended_map(goal(1), goal(2)) == 1
    path = [];
    return;
end

open_set = false(rows, cols);
closed_set = false(rows, cols);
g_cost = inf(rows, cols);
f_cost = inf(rows, cols);
parent = zeros(rows, cols, 2);


g_cost(start(1), start(2)) = 0;
f_cost(start(1), start(2)) = heuristic(start, goal);
open_set(start(1), start(2)) = true;

directions = [-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];

while any(open_set(:))
    masked_f_cost = f_cost;
    masked_f_cost(~open_set) = inf;
    [~, min_idx] = min(masked_f_cost(:));

    [cx, cy] = ind2sub(size(f_cost), min_idx);

    if cx == goal(1) && cy == goal(2)
        path = reconstruct_path(parent, goal);
        return;
    end

    open_set(cx, cy) = false;
    closed_set(cx, cy) = true;

    for i = 1:size(directions, 1)
        nx = cx + directions(i, 1);
        ny = cy + directions(i, 2);

        if nx < 1 || ny < 1 || nx > rows || ny > cols
            continue;
        end

        if extended_map(nx, ny) == 1 || closed_set(nx, ny)
            continue;
        end

        t_g_cost = g_cost(cx, cy) + norm([nx - cx, ny - cy]);

        if ~open_set(nx, ny) || t_g_cost < g_cost(nx, ny)
            parent(nx, ny, :) = [cx cy];
            g_cost(nx, ny) = t_g_cost;
            f_cost(nx, ny) = t_g_cost + heuristic([nx ny], goal);
            open_set(nx, ny) = true;
        end
    end
end

path = [];


end

