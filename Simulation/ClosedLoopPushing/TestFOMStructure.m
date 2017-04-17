%% Clear
clear all;
close all;
clc;
%% Setup
run('Setup.m');
import Models.QSPusherSlider
import MPCSolvers.FOMSolver
import Combinatorics.VariationWithRepetition
DateString = datestr(clock, 'mm_dd_yyyy__HH_MM_SS');
%% Optimization Hybrid States Set Up
c2 = QSPusherSlider.c^2;
hybrid_states_map = horzcat(PusherSliderStates.QSSticking(QSPusherSlider.a, QSPusherSlider.nu_pusher, c2), ...
                            PusherSliderStates.QSSlidingUp(QSPusherSlider.a, QSPusherSlider.nu_pusher, c2), ...
                            PusherSliderStates.QSSlidingDown(QSPusherSlider.a, QSPusherSlider.nu_pusher, c2));
c2_pert = QSPusherSlider.c_pert^2;
%% Euler Integration Hybrid States Set Up
real_states_map = horzcat(PusherSliderStates.QSSticking(QSPusherSlider.a, QSPusherSlider.nu_pusher_pert, c2_pert), ...
                          PusherSliderStates.QSSlidingUp(QSPusherSlider.a, QSPusherSlider.nu_pusher_pert, c2_pert), ...
                          PusherSliderStates.QSSlidingDown(QSPusherSlider.a, QSPusherSlider.nu_pusher_pert, c2_pert));
%% Hybrid Modes Parameters and Setup
fom_steps = 5;
hybrid_modes = VariationWithRepetition(length(hybrid_states_map), fom_steps);
%% MPC Parameters and Setup
[Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt, h_step] = GetTestParams();
solver = FOMSolver(hybrid_states_map, Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt, hybrid_modes, 0); % FOM without chameleon
%% Solve MPC and Integrate
tf = fom_steps * h_opt; % We only take one step of the Euler integration
u_thrust = 0.05;
u_star = @(t)([u_thrust * ones(size(t)); 0 .*t]);
x_star = @(t)([u_thrust .* t; 0.*t; 0.*t; 0.*t]); % For the straight line
eps = 0.5; % TODO: Automatically generate it
% eps = 0.1 * tf * u_thrust; % 10% of the path if it were a straight line
angular_eps = pi / 4; % 45% in each direction. Otherwise we choose another side
sim_number = 100; % Number of simulations
n_rand = @(n, m, eps)((rand(n, m) - 0.5) * eps);
x0 = [n_rand(sim_number, 2, 2 * eps), n_rand(sim_number, 1, 2 * angular_eps), n_rand(sim_number, 1, QSPusherSlider.a)].';
path_name = 'SimulationResults/TestFOMStructure';
if  exist(path_name, 'dir') ~= 7
    mkdir(pwd, path_name);
end
% fileID = fopen(strcat(path_name, '/cost_data_', sim_number, '.csv'));
for i = 1:sim_number
    disp(i)
    for j = 1:size(hybrid_modes, 1)
        op = solver.GetOptimizationProblem(0, x_star, u_star, x0(:,i), hybrid_modes(j, :));
        [~, ~, cost] = op.solve;
        if (~exist('data', 'var'))
            data = [i, j, cost];
        else
            data = [data; i, j, cost];
        end
    end
end
csvwrite(strcat(path_name, '/cost_data_', int2str(sim_number), '.csv'), data);
