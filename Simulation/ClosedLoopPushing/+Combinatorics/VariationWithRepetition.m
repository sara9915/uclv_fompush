function [ result ] = VariationWithRepetition(n, m)
    result = PrivateVariationWithRepetition(n, m, 1, zeros(1, m));
end

function [result] = PrivateVariationWithRepetition(n, m, i, local)
    result = [];
    if i == m + 1
        result = [result; local];
    else
        for j = 1:n
            local(i) = j;
            result = [result; PrivateVariationWithRepetition(n, m, i + 1, local)];
        end
    end
end