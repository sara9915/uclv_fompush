%% Clear
clear all;
close all;
clc;
%% Setup
run('Setup.m');
import Models.QSPusherSlider
import MPCSolvers.FOMSolver
import Combinatorics.VariationWithRepetition
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
Q_MPC = 10 * diag([1,1,.1,0]); % 10 * diag([1,10,10,0]);
Q_MPC_final = 200 * diag([1,1,.1,0]); % 200 * diag([.1,10,1,0]);
R_MPC = .5 * diag([1,1]);
u_lower_bound = [-0.01; 0.1];
u_upper_bound = [0.1; 0.1];
% x_lower_bound = [1; 1; 20; 100];
% x_upper_bound = [1; 1; 20; 100];
x_lower_bound = [1; 1; 20; QSPusherSlider.a];
x_upper_bound = [1; 1; 20; QSPusherSlider.a];
h_opt = 0.03;
solvers{1, 1} = FOMSolver(hybrid_states_map, Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt, hybrid_modes, 0); % FOM without chameleon
solvers{1, 2} = FOMSolver(hybrid_states_map, Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt, hybrid_modes, 1); % FOM with chameleon
%% Euler Integration Parameters and Setup
h_step = 0.01;
euler_integrator = EulerIntegration(real_states_map, h_step);
%% Solve MPC and Integrate
t0 = 0;
tf = h_step;
u_thrust = 0.05;
u_star = @(t)([u_thrust * ones(size(t)); 0 .*t]); % To ensure it can be substituted by a vector
x_star = @(t)([u_thrust .* t; 0.*t; 0.*t; 0.*t]); % For the straight line
% d = 0.5 * QSPusherSlider.b/2.0;
% c = QSPusherSlider.c;
% px = QSPusherSlider.a / 2.0;
% A = u_thrust / (c^2 + px^2 + d^2);
% x_star = @(t)([-(sin(-(d*A).*t) * (c^2 + px^2) + cos(-(d*A) .* t) * px*d - px*d) / d; (cos(-(d*A).*t) * (c^2 + px^2) - sin(-(d*A).* t) * px*d - c^2 - px^2) / d; -(d*A) .* t; d * ones(size(t))]); % For the circle
% x0 = [0, .05, 30 * pi / 180, 0]; % For the straight line
% x0 = [0, 0.05, 0, 0]; % For the circle
x0 = [(rand(1000,2) - 0.5) * 1, rand(1000,1) * pi, (rand(1000,1) - 0.5) * QSPusherSlider.a];
path_name = 'SimulationResults/TestFOMStructure';
if  exist(path_name, 'dir') ~= 7
    mkdir(pwd, path_name);
end
for i = 1:length(solvers)
    [x_state, u_state, x_bar, u_bar, t, modes, costs] = euler_integrator.IntegrateMPC(t0, tf, x0, x_star, u_star, solvers{i});
end
