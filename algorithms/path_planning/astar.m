function [path] = astar(read_only_vars, public_vars)

extended_map = extend_map(read_only_vars.discrete_map.map, public_vars.walls_width);

[rows, cols] = size(extended_map);

robot_est_pos = [public_vars.estimated_pose(1) public_vars.estimated_pose(2)];
robot_disc_x_pos = round(((robot_est_pos(1) - read_only_vars.map.limits(1))/(read_only_vars.map.limits(3) - read_only_vars.map.limits(1)))*read_only_vars.discrete_map.dims(1));
robot_disc_y_pos = round(((robot_est_pos(2) - read_only_vars.map.limits(2))/(read_only_vars.map.limits(4) - read_only_vars.map.limits(2)))*read_only_vars.discrete_map.dims(2));

robot_discrete_pos = [robot_disc_x_pos robot_disc_y_pos];

open_set = false(rows, cols);
closed_set = false(rows, cols);

g_cost = inf(rows, cols);
f_cost = inf(rows, cols);

parent = zeros(rows, cols, 2);

sx = robot_discrete_pos(1);
sy = robot_discrete_pos(2);

if extended_map(sx, sy) == 1
    path = [];
    return;
end

gx = read_only_vars.discrete_map.goal(1);
gy = read_only_vars.discrete_map.goal(2);

if extended_map(gx, gy) == 1
    path = [];
    return;
end

g_cost(sx, sy) = 0;
f_cost(sx, sy) = heuristic([sx sy], [gx gy]);
open_set(sx, sy) = true;

directions = [-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];

while any(open_set(:))
    % [min_val min_idx] = min(f_cost(:).*open_set(:) + inf*(~open_set(:)));
    masked_f_cost = f_cost;
    masked_f_cost(~open_set) = inf;
    [min_val, min_idx] = min(masked_f_cost(:));

    [cx cy] = ind2sub(size(f_cost), min_idx);

    if cx == gx && cy == gy
        path = reconstruct_path(parent, read_only_vars.discrete_map.goal);
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
            f_cost(nx, ny) = t_g_cost + heuristic([nx ny], [gx gy]);
            open_set(nx, ny) = true;
        end
    end
end

path = [];


end

