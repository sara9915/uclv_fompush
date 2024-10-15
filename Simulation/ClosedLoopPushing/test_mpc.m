% clear variables
% close all
% clc

data = readtable("mpc_debugger_msg_last.csv");
start_sample = 1;
end_sample = 300;
t = data.x__time - data.x__time(1);
t = t(start_sample:end_sample);


%% Get x y theta slider in inertial frame
thetai = data.x_debugger_mpc_pose_theta;
thetai = thetai(start_sample:end_sample);
xi = data.x_debugger_mpc_pose_x; 
xi = xi(start_sample:end_sample);
yi = data.x_debugger_mpc_pose_y;
yi = yi(start_sample:end_sample);
% PLOT 
figure(1)
ax1 = subplot(3,1,1);
plot(t,thetai), grid on, title("theta")
ax2 = subplot(3,1,2);
plot(t,xi), grid on, title("x")
ax3 = subplot(3,1,3);
plot(t,yi), grid on, title("y")
linkaxes([ax1,ax2, ax3],'x');


%% Get rx ry (pusher coordinates in body frame)
rxi = data.x_debugger_pusher_body_x(start_sample:end_sample);
% rxi = -0.0410*ones(length(rxi),1);%ri(:,1);
ryi = data.x_debugger_pusher_body_y(start_sample:end_sample);
ri = [rxi ryi];


% PLOT 
figure(5)
ax5 = subplot(2,1,1);
plot(t,rxi), grid on, title("rxb")
ax6 = subplot(2,1,2);
plot(t,ryi), grid on, title("ryb")
linkaxes([ax5,ax6],'x');

% get x_state
x_state_cpp = [xi, yi, thetai, rxi, ryi];


%% Get u control body mpc
u_control_xb = data.x_debugger_u_control_linear_x(start_sample:end_sample);
u_control_yb = data.x_debugger_u_control_linear_y(start_sample:end_sample);
u_controlb = [u_control_xb, u_control_yb];

% PLOT
figure(10)
ax10 = subplot(2,1,1);
plot(t,u_control_xb), title("uxb normale mpc"), grid on
ax11 = subplot(2,1,2);
plot(t,u_control_yb), title("uyb tangenziale mpc"), grid on
linkaxes([ax10,ax11],'x');

%% Get vipi
u_control_i = [];

for i=1:length(u_control_xb)
%     tmp = rotz(thetai(i)*180/pi) * [u_controlb(i,:) 0]';
    Cbi = Helper.C3_2d(thetai(i));
    tmp = Cbi' * u_controlb(i,:)';
    u_control_i = [u_control_i; tmp(1:2)'];
end

% PLOT
figure(11)
ax10 = subplot(2,1,1);
plot(t,u_control_i(:,1)), title("uxi normale mpc"), grid on
ax11 = subplot(2,1,2);
plot(t,u_control_i(:,2)), title("uyi tangenziale mpc"), grid on
linkaxes([ax10,ax11],'x');


%% Get control modes
deltau_st_xb = data.x_debugger_u_control_st_linear_x(start_sample:end_sample);
deltau_st_yb = data.x_debugger_u_control_st_linear_y(start_sample:end_sample);
deltau_st = [deltau_st_xb, deltau_st_yb];

deltau_su_xb = data.x_debugger_u_control_su_linear_x(start_sample:end_sample);
deltau_su_yb = data.x_debugger_u_control_su_linear_y(start_sample:end_sample);
deltau_su = [deltau_su_xb,deltau_su_yb];

deltau_sd_xb = data.x_debugger_u_control_sd_linear_x(start_sample:end_sample);
deltau_sd_yb = data.x_debugger_u_control_sd_linear_y(start_sample:end_sample);
deltau_sd = [deltau_sd_xb, deltau_sd_yb];

% PLOT 
figure(500)
ax500 = subplot(3,1,1);
plot(deltau_st), grid on, title("deltau st"), legend("normale","tangenziale")
ax502 = subplot(3,1,2);
plot(deltau_su), grid on, title("deltau su"),  legend("normale","tangenziale")
ax503 = subplot(3,1,3);
plot(deltau_sd), grid on, title("deltau sd"),  legend("normale","tangenziale")
linkaxes([ax500,ax502, ax503],'x');

