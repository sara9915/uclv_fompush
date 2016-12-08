%% Author: Francois Hogan
%% Date: 12/08/2016

%% Clear
clear all;
close all;
clc;
%Initialize Drake
%run('/Users/Francois/Dropbox (MIT)/Matlab/MIT/Drake/drake/addpath_drake');
run('~/gelsight/software/externals/drake/addpath_drake');
%%Add path
%addpath(fullfile('/Users/Francois/Dropbox (MIT)/Matlab/MIT/System'));
addpath(fullfile('~/testscripts/System'));

%% Simulation Parameters
t0 = 0;
tf = 5;
h_step = 0.01;

tic;
%% Build PusherSlider
p = PusherSlider('Trajectory');%'Target'

%% Save data in new folder
p.SimName = 'SimulationResultsTest'; %Maybe add date, or even date and time, so its not overwritten
mkdir(pwd,p.SimName);
p.FilePath = strcat(pwd,'/',p.SimName);
FileName = strcat(p.FilePath,'/',p.SimName);

%% Linearize and build constraint matrices
p.symbolicLinearize();
p.symbolicNonLinearize();
p.ConstraintMatrices('FOM');

%% Simulation Parameters
%Define number of simulations to perform
p.NumSim = 1;

%% Simulate Forward
%Initial condition for 5 different simulations 
%Define d0
vecD0{1} = 0;
% vecD0{2} =  0.01;
% vecD0{3} = -0.03;
% vecD0{4} =  0.01;
% vecD0{5} =  0.00;
%Define qs0
vecQS0{1} = [0;0.05;30*pi/180];
% vecQS0{2} = [-0.02;   .05; 20*pi/180];
% vecQS0{3} = [0.00;    0  ; 15*pi/180];
% vecQS0{4} = [-0.06; -.05  ; -30*pi/180];
% vecQS0{5} = [0.00;   -0.1 ; -130*pi/180];

for lv1=1:p.NumSim
    p.EulerIntegration( t0, tf, h_step, vecD0{lv1}, vecQS0{lv1}, lv1);
end

p.Animate(1);
toc
%% Post-Processing
save(FileName, 'p');

