addpath('../../software/externals/jsonlab');
run('../../software/externals/drake/addpath_drake');
addpath(fullfile('../System'));
if isunix()
    addpath('/home/mcube/software/gurobi701/linux64/matlab');
    gurobi_setup;
elseif ispc()
    addpath('C:\gurobi701\win64\matlab');
end
    