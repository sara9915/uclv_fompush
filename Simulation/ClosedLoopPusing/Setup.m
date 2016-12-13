addpath('../../software/externals/jsonlab');
if isunix()
    addpath('/home/mcube/software/gurobi651/linux64/matlab');
    gurobi_setup;
elseif ispc()
    addpath('C:\gurobi701\win64\matlab');
end
    