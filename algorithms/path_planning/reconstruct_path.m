function [path] = reconstruct_path(parent, goal)

x = goal(1);
y = goal(2);

path = [x y];

while any(parent(x, y, :))
    px = parent(x, y, 1);
    py = parent(x, y, 2);

    path = [px py; path];
    x = px;
    y = py;
end

end