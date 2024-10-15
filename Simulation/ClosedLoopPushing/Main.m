%% Author: Eudald Romo
%% Date: 12/22/2016

%% Clear
clear variables; % Clear all is bad for performance
% close all;
clc;
% Setup
run('Setup.m');

%% Simulation Parameters
t0 = 0;
tf = 40;
h_step = 0.050;

tic;
%% Build PusherSlider
p = PusherSlider('Target');%'Target'

%% Save data in new folder
SimName = 'SimulationResults'; %Maybe add date, or even date and time, so its not overwritten
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

%% Test MPC matlab - c++
% test_mpc
%
% vipi_cpp = [];
% vbpi_cpp = [];
%
% for k=1:length(t)
%     tic
%     vipi = p.controller(t(k)+h_step*p.sample_delay, x_state_cpp(k,:));
%     toc
%     vipi_cpp = [vipi_cpp;vipi'];
%     Cbi = Helper.C3_2d(x_state_cpp(k,3));
%     vbpi = Cbi*vipi;
%     vbpi_cpp = [vbpi_cpp;vbpi'];
% end
%
% figure, plot(vbpi_cpp(:,1)), hold on, plot(u_controlb(:,1)), hold off, grid on, legend("matlab", "cpp"), title("Normali")
% ax = gca;
% figure, plot(vbpi_cpp(:,2)), hold on, plot(u_controlb(:,2)), hold off, grid on, legend("matlab", "cpp"), title("Tangenziali")
% ax(end+1) = gca;
%
% figure, plot(p.deltau_st(:,1)), hold on, plot(deltau_st(:,1)), hold off, grid on, legend("matlab", "cpp"), title("Sticking Normali")
% ax(end+1) = gca;
% figure, plot(p.deltau_st(:,2)), hold on, plot(deltau_st(:,2)), hold off, grid on, legend("matlab", "cpp"), title("Sticking Tangenziali")
% ax(end+1) = gca;
%
% figure, plot(p.deltau_su(:,1)), hold on, plot(deltau_su(:,1)), hold off, grid on, legend("matlab", "cpp"), title("Sliding up Normali")
% ax(end+1) = gca;
% figure, plot(p.deltau_su(:,2)), hold on, plot(deltau_su(:,2)), hold off, grid on, legend("matlab", "cpp"), title("Sliding up Tangenziali")
% ax(end+1) = gca;
%
% figure, plot(p.deltau_sd(:,1)), hold on, plot(deltau_sd(:,1)), hold off, grid on, legend("matlab", "cpp"), title("Sliding down Normali")
% ax(end+1) = gca;
% figure, plot(p.deltau_sd(:,2)), hold on, plot(deltau_sd(:,2)), hold off, grid on, legend("matlab", "cpp"), title("Sliding down Tangenziali")
% ax(end+1) = gca;
%
% linkaxes(ax,'x');
%% Simulate Forward
% t_dist = [4/0.05  7/0.05];
t_dist = [15/0.05 27/0.05];

x0_x = [0.0260    0.0107    0.0155    0.0146   -0.0065];
x0_y = [0.0093   -0.0197    0.0124   -0.0281   -0.0134];
x0_theta = [-8.0492   -4.4300    0.9376    9.1501    9.2978]; %degree
x0_Spy = [-0.0196  -0.0065    0.0212    0.0096    0.0273];
y_dist = [0.0315    0.0406   -0.015   0.0001    0.0132];

%Initial conditions
% vecD0 =  [0.0 %-0.03 ... %ry
%           0.0
%         ... .01, -.03, ...
%         ... .01, .00 ...
%         ];
% vecQS0 = [0.000    -0.0    deg2rad(0)
%           0.0 0.0 deg2rad(0)  % x y theta
%             ... 0, .05, 30 * pi / 180; ...
%             ... -.02, .05, 20 * pi / 180; 0, 0, 15 * pi / 180; ...
%             ... -.06, -.05, -30 * pi / 180; 0, -.1, -130 * pi / 180 ...
%             ];

% for lv1=1:p.NumSim
%     p.EulerIntegration(t0, tf, h_step, vecD0(lv1), %vecQS0(lv1,:).', lv1);
% end

vecD0 = x0_Spy;

current_sim = 25;
total_sim = length(t_dist) * length(x0_x) * length(y_dist);

for index_t = 2 : length(t_dist)
    if index_t == 1
        y_dist = [-0.035   -0.0161   -0.0127   -0.0021   -0.0018];
    else
        y_dist = [0.0215    -0.0306   -0.015   0.017    0.0132];
    end
    for index_x0 = 1 : length(x0_x)
        for index_ydist = 1 : length(y_dist)
            t0 = 0;
            p = PusherSlider('Target');%'Target'
            p.symbolicLinearize();
            p.symbolicNonLinearize();
            
            p.ConstraintMatrices('FOM');
            current_sim = current_sim + 1;
            disp(strcat("START SIMULATION: ", num2str(current_sim), "/", num2str(total_sim)));
            filename = strcat(num2str(current_sim),"_traj1_",num2str(index_t),"_",num2str(index_x0),"_",num2str(index_ydist));
            p.EulerIntegration(t0, tf, h_step, x0_Spy(index_x0), [x0_x(index_x0) x0_y(index_x0) deg2rad(x0_theta(index_x0))]',1, filename, t_dist(index_t), y_dist(index_ydist));
        end
    end
end
return

% p.Animate(file_name, 1);
animator = Animator;
% animator.Animate(p, file_name, 1)
%% PLOT
% figure, plot(p.delta_u_pert{1})
% close all
figure, plot(p.u_state{2}), grid on, legend('u normale', 'u tangenziale')
figure(51)
ax1 = subplot(2,2,1);
plot(p.x_state{2}(:,1)), grid on, hold on, plot(p.x_star_vect(1,:)), hold off, legend('x','x*'), title('x')
ax2 = subplot(2,2,2);
plot(p.x_state{2}(:,2)), grid on, hold on, plot(p.x_star_vect(2,:)), hold off, legend('y','y*'), title('y')
ax3 = subplot(2,2,3);
plot(p.x_state{2}(:,3)), grid on, hold on, plot(p.x_star_vect(3,:)), hold off, legend('theta','theta*'), title('theta')
ax4 = subplot(2,2,4);
plot(p.x_state{2}(:,5)), grid on, hold on, plot(p.x_state{1}(:,5)), hold off, legend('ry','ry*'), title('ry')
linkaxes([ax1,ax2,ax3,ax4],'x');
% figure, plot(p.x_state{2}), grid on, hold on,  plot(p.x_state{1}),legend('x','y','theta','rx','ry','x*','y*','theta*','rx*','ry*'),hold off
figure(101)
ax101 = subplot(2,1,1);
plot(p.solutionVec), grid on, legend('sticking', 'sliding up', 'sliding down')
ax102 = subplot(2,1,2);
plot(p.index,'*'), grid on
linkaxes([ax101,ax102],'x');
figure, plot(p.x_state{2}(1:length(p.x_star_vect(3,:)),1:3)-p.x_star_vect'), grid on, legend('error x','error y','error theta','error rx','error ry')
toc

return
%% Post-Processing
save('robotics.mat', 'p');

%% COMPARISON

% close all
clc

load('ral_disturbance.mat')
load('mdpi_disturbance.mat')

figure
subplot(3,1,1), plot(mdpi.x(1:length(p.x_star_vect(3,:)))-p.x_star_vect(1,:)'), hold on, plot(params.x_S(1:length(params.controller.y_ref(1,:)))-params.controller.y_ref(1,:)), grid on, title('x-error'), legend('FOM','NMPC'), ylabel('error_x [m]'), xlabel('samples')
subplot(3,1,2), plot(mdpi.y(1:length(p.x_star_vect(3,:)))-p.x_star_vect(2,:)'), hold on, plot(params.y_S(1:length(params.controller.y_ref(1,:)))-params.controller.y_ref(2,:)), grid on, title('y-error'), legend('FOM','NMPC'), ylabel('error_y [m]'), xlabel('samples')
subplot(3,1,3), plot(mdpi.theta(1:length(p.x_star_vect(3,:)))-p.x_star_vect(3,:)'), hold on, plot(params.theta_S(1:length(params.controller.y_ref(1,:)))-params.controller.y_ref(3,:)), grid on, title('theta-error'), legend('FOM','NMPC'), ylabel('error_{\theta} [m]'), xlabel('samples')



figure,
subplot(2,1,1), plot(p.u_state{2}(:,1)), hold on, plot(params.u_n), grid on, title('normal velocity'), legend('FOM','NMPC')
subplot(2,1,2), plot(p.u_state{2}(:,2)), hold on, plot(params.u_t), grid on, title('tangential velocity'), legend('FOM','NMPC')

rms_fom_state = [rms(p.x_state{2}(1:length(p.x_star_vect(3,:)),1)-p.x_star_vect(1,:)'), rms(p.x_state{2}(1:length(p.x_star_vect(3,:)),2)-p.x_star_vect(2,:)'), rms(p.x_state{2}(1:length(p.x_star_vect(3,:)),3)-p.x_star_vect(3,:)')];
rms_nmpc_state = [rms(params.x_S(1:length(params.controller.y_ref(1,:)))-params.controller.y_ref(1,:)), rms(params.y_S(1:length(params.controller.y_ref(1,:)))-params.controller.y_ref(2,:)), rms(params.theta_S(1:length(params.controller.y_ref(1,:)))-params.controller.y_ref(3,:))];

rms_fom_controller = [rms(p.u_state{2}(:,1)) rms(p.u_state{2}(:,2))];
rms_nmpc_controller = [rms(params.u_n) rms(params.u_t)];

% Evaluate cost function
W_x = 1*diag([100 100 0.1 0]);
W_x_e = 200*diag([1000 1000 0.1 0]);
W_u = diag([1e-3 1e-3]);
length_ = length(p.x_star_vect(3,:));
x_state_ral = [params.x_S(1:length_); params.y_S(1:length_); params.theta_S(1:length_); params.S_p_y(1:length_)];
u_ral = [params.u_n(1:length_); params.u_t(1:length_)];

x_state_mdpi = [mdpi.x(1:length_) mdpi.y(1:length_) mdpi.theta(1:length_) mdpi.rx_ry(1:length_,1)]';
u_mdpi = p.u_state{2}(1:length_,:);

x_ref_ = [p.x_star_vect(1,:)' p.x_star_vect(2,:)' p.x_star_vect(3,:)' zeros(length(p.x_star_vect(3,:)),1)]';
u_ref_ = ones(length_,2);
u_ref_(:,1) = p.u_star(1);

cost_function_ral = [];
cost_function_mdpi = [];
for i = 1:length_
    cost_function_ral = [cost_function_ral (x_ref_(:,i)-x_state_ral(:,i))'*W_x*(x_ref_(:,i)-x_state_ral(:,i) + (u_ref_(i,:)'-u_ral(:,i))'*W_u*(u_ref_(i,:)'-u_ral(:,i)))];
    cost_function_mdpi = [cost_function_mdpi (x_ref_(:,i)-x_state_mdpi(:,i))'*W_x*(x_ref_(:,i)-x_state_mdpi(:,i))];% + (u_ref_(i,:)'-u_ral(:,i))'*W_u*(u_ref_(i,:)'-u_ral(:,i)))];
end

figure, plot(cost_function_mdpi), hold on, plot(cost_function_ral), grid on, legend('FOM','NMPC')

