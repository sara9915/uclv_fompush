addpath('../../software/externals/jsonlab');
addpath(fullfile('../System'));
if isunix()
    addpath(strcat(getenv('GUROBI_HOME'),'/matlab'));
    gurobi_setup;
elseif ispc()
    addpath('C:\gurobi701\win64\matlab');
end
    