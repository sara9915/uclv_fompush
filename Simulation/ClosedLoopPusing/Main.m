%% Author: Eudald Romo
%% Date: 12/22/2016

%% Clear
clear variables; % Clear all is bad for performance
close all;
clc;
% Setup
run('Setup.m');

%% Simulation Parameters
t0 = 0;
tf = 5;
h_step = 0.01;

tic;
%% Build PusherSlider
p = PusherSlider('Trajectory');%'Target'

%% Save data in new folder
SimName = 'SimulationResultsTest'; %Maybe add date, or even date and time, so its not overwritten
if  exist(SimName, 'dir') ~= 7
    mkdir(pwd, SimName);
end
FilePath = strcat(pwd, '/', SimName);
file_name = strcat(FilePath, '/', SimName);

%% Linearize and build constraint matrices
p.symbolicLinearize();
p.symbolicNonLinearize();
p.ConstraintMatrices('FOM');

%% Simulation Parameters
%Define number of simulations to perform
p.NumSim = 1;

%% Simulate Forward
%Initial conditions
vecD0 = [0, ...
        ... .01, -.03, ...
        ... .01, .00 ...
        ];
vecQS0 = [0, .05, 30 * pi / 180; ...
            ... -.02, .05, 20 * pi / 180; 0, 0, 15 * pi / 180; ...
            ... -.06, -.05, -30 * pi / 180; 0, -.1, -130 * pi / 180 ...
            ];

for lv1=1:p.NumSim
    p.EulerIntegration(t0, tf, h_step, vecD0(lv1), vecQS0(lv1,:).', lv1);
end
% p.Animate(file_name, 1);
animator = Animator;
animator.Animate(p, file_name, 1)
toc
%% Post-Processing
save(file_name, 'p');

