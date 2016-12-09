%% Plots results
clear
clc
close all

%Initialize Drake and System
run('../../software/externals/drake/addpath_drake');
addpath(fullfile('../System'));

%% Enter name of JSON file below in Filename
Filename = '10Rings_151g_ClosedLoopStraight02';
Foldername = 'Results/Experiments_2016_11_23/Data/';
Json = loadjson(strcat(Foldername,Filename,'.json'));

%% Build Pusher Object
p = PusherSlider('Target');
p.symbolicLinearize();
p.symbolicNonLinearize();
p.ConstraintMatrices('FOM');
p.NumSim = 1;

%% Initialize name of files and folders
p.SimName = Filename;
mkdir(pwd,p.SimName);
p.FilePath = strcat(pwd,'/',p.SimName);
FileName = strcat(p.FilePath,'/',p.SimName);
%% Load Experiment Data
Json.q_sliderJSON(1,:) = Json.q_sliderJSON(1,:) - 0.15*0;
Json.q_pusher_sensedJSON(1,:) = Json.q_pusher_sensedJSON(1,:) - 0.15*0;
MaxLength = length(Json.timeJSON);
p.t = Json.timeJSON(:,1:MaxLength);
p.x_state{2} = [Json.q_sliderJSON(:,1:MaxLength)' Json.q_pusher_sensedJSON(:,1:MaxLength)'];
p.u_state{2} = [Json.q_sliderJSON(:,1:MaxLength)' Json.q_pusher_sensedJSON(:,1:MaxLength)'];

%% Animation
tf = Json.timeJSON(end);
N  = length(Json.timeJSON);
x0 = Json.q_sliderJSON(1,1);

for lv2=1:N
    index = lv2;
    t0 = Json.timeJSON(1,index);
    
    delta_uMPC = Json.delta_uMPCJSON(:,index);
    delta_xMPC = Json.delta_xMPCJSON(:,index);
    delta_xMPC = reshape(delta_xMPC, [4, 35]);

    for lv1=1:35
        time(lv1)  = t0 + lv1*0.03;
        x_des = x0+0.05*(time(lv1)-1);
        %
        x_nom(lv1)     = x_des;
        y_nom(lv1)     = 0;
        theta_nom(lv1) = 0;
        py_nom(lv1) = 0;
        %
        delta_x(lv1)     = delta_xMPC(1,lv1);
        delta_y(lv1)     = delta_xMPC(2,lv1);
        delta_theta(lv1) = delta_xMPC(3,lv1);
        delta_py(lv1) = delta_xMPC(4,lv1);
        %
        x(lv1)     = x_nom(lv1) + delta_x(lv1);
        y(lv1)     = y_nom(lv1) + delta_y(lv1);
        theta(lv1) = theta_nom(lv1) + delta_theta(lv1);
        py(lv1)    = py_nom(lv1)+ delta_py(lv1);
    end
     xMPC{lv2}=x-0.0985/2;
     yMPC{lv2}=y;
     thetaMPC{lv2}=theta;
     pyMPC{lv2}=py;
end

p.AnimateMPC(xMPC, yMPC, thetaMPC, pyMPC);

%% Generate Data
Data.xMPC = xMPC;
Data.yMPC = yMPC;
Data.thetaMPC = thetaMPC;
Data.t = p.t-1;
Data.x = Json.q_sliderJSON(1,:);
Data.y = Json.q_sliderJSON(2,:);
Data.theta = Json.q_sliderJSON(3,:);

save(strcat(Foldername, Filename), 'Data');


