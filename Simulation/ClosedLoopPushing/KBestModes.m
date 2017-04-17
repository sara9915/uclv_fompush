function [mem, father, costs] = KBestModes(cost_matrix, k)
% Gets k best modes for a nxm matrix cost_matrix with cost(n * m^2 * k)
[n, m] = size(cost_matrix);
mem = cell(m, k);
father = cell(m, k);
costs = Inf * ones(m, k);
for rem = 1:k
    for i = 1:m
        current_cost = cost_matrix(:, i);
        if rem == 1
            mem{i, rem} = current_cost;
            father{i, rem} = [i, rem];
            costs(i, rem) = sum(current_cost) / n;
        elseif i < rem
            mem{i, rem} = Inf * ones(n, 1);
        else
            loc_min = Inf;
            best_vec = Inf * ones(n, 1);
            for j = 1:(i-1)
                curr_min = min(current_cost, mem{j, rem -1});
                if sum(curr_min) < loc_min
                    loc_min = sum(curr_min);
                    best_vec = curr_min;
                    father{i, rem} = [j, rem - 1];
                end
            end
            mem{i, rem} = best_vec;
            costs(i, rem) = sum(best_vec) / n;
        end
    end
end
end