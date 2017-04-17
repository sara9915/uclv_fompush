function [Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt, h_step] = GetTestParams()

import Models.QSPusherSlider
%% MPC Parameters and Setup
Q_MPC = 1 * diag([10,10,.1,0]); % 10 * diag([1,10,10,0]);
Q_MPC_final = 100 * diag([1,1,.1,0]); % 200 * diag([.1,10,1,0]);
R_MPC = 0 * diag([1,1]);
u_lower_bound = [-0.01; 0.1];
u_upper_bound = [0.1; 0.1];
x_lower_bound = [1; 1; 20; QSPusherSlider.a];
x_upper_bound = [1; 1; 20; QSPusherSlider.a];
h_opt = 0.03;
h_step = 0.01;
end

