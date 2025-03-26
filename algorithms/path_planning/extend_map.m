function [extended_map] = extend_map(map, clearance)

[rows, cols] = size(map);

extended_map = map;

for i = 1:rows
    for j = 1:cols
        if map(i,j) == 1
            for k = max(1, i - clearance):min(rows, i + clearance)
                for l = max(1, j - clearance):min(cols, j + clearance)
                    if sqrt((i - k)^2 + (j - l)^2) <= clearance
                        extended_map(k,l) = 1;
                    end
                end
            end
        end
    end
end

end