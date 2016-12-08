function obj = ConstraintMatrices(obj, flag)
    if strcmp(flag,'FOM')
        obj.FOM = 1;
        obj.ConstraintMatricesFOM();
    elseif strcmp(flag,'MIQP')
        obj.MIQP = 1;
        obj.ConstraintMatricesMIQP();
    else
        disp('Error: Could not find proper flag string');
    end
end