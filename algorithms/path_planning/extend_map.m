function [extended_map] = extend_map(map, clearance)

[rows, cols] = size(map);
extended_map = map;

for i = 1:rows
    for j = 1:cols
        if map(i,j) == 1
            row_min = max(1, i - clearance);
            row_max = min(rows, i + clearance);
            col_min = max(1, j - clearance);
            col_max = min(cols, j + clearance);

            extended_map(row_min:row_max, col_min:col_max) = 1;
        end
    end
end

end