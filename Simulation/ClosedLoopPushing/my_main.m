clear variables
close all
clc

%% MPC INITIALIZATION
% Setup
run('Setup.m');

% Simulation Parameters
t0 = 0;
tf = 60;
h_step = 0.030;

tic;
% Build PusherSlider
p = PusherSlider('Target');%'Target'

% Save data in new folder
SimName = 'SimulationResults'; %Maybe add date, or even date and time, so its not overwritten
if  exist(SimName, 'dir') ~= 7
    mkdir(pwd, SimName);
end
FilePath = strcat(pwd, '/', SimName);
file_name = strcat(FilePath, '/', SimName);

% Linearize and build constraint matrices
p.symbolicLinearize();
p.symbolicNonLinearize();

p.ConstraintMatrices('FOM');

%Define number of simulations to perform
p.NumSim = 1;

%% ROS INITIALIZATION

rosinit('192.168.2.94',11310)


has_robot = false;
has_mpc_pose = false;

% tf object
tftree = rostf;
tftree.BufferTime = 0.5;
pause(2);

% Get homogeneous transform from base_link to slider0
T_BS0_ = getTransform(tftree, "slider0", "base_link",rostime(0));
T_BS0 = transformToMatrix(T_BS0_);

% Dealy Buffer
global sample_command_delay_buffer;
sample_command_delay_buffer = 5;%3;
global command_delay_buffer
command_delay_buffer = zeros(2,sample_command_delay_buffer);%obj.sample_delay);
command_delay_buffer(1,:) = p.u_star(1);
command_delay_buffer(2,:) = p.u_star(2);

command_pub = rospublisher("/command_vel_des","geometry_msgs/Twist");
command_msg = rosmessage(command_pub);
start_time = rostime('now');
p.start_time_ros = start_time;
mpc_state_sub = rossubscriber("/mpc_pose","geometry_msgs/Pose2D", {@get_mpc_state,tftree,T_BS0, p, start_time, command_pub, command_msg}, "BufferSize",1);
global point_sl 
point_sl = 0.30;

function get_mpc_state(mpc_state_sub, mpc_pose, tftree, T_BS0, p, start_time, command_pub, command_msg)
    global command_delay_buffer sample_command_delay_buffer point_sl
    tic
    t = rostime('now')-start_time;
    if isinf(p.flag) %t.seconds > point_sl/PusherSlider.u_star(1) 
        command_msg.Linear.X = 0;
        command_msg.Linear.Y = 0;
        send(command_pub,command_msg)
        pause(2);
        rosshutdown
        return
    end
    
    % Get homogeneous transform from base_link to push_frame
    T_PB_ = getTransform(tftree, "base_link", "push_frame",rostime(0));
    T_PB = transformToMatrix(T_PB_);
    
%     T_PB(1,end) = -p.a/2;
    
    % Get homogeneous transform from push_frame to slider0
    T_PS0 = T_BS0 * T_PB;
    T_PS0(2,end) = T_PS0(2,end);
    
    
    Rbi_tmp = Helper.C3_2d(mpc_pose.Theta);
    ripi_tmp = [T_PS0(1,end);T_PS0(2,end)];
    ribi_tmp = [mpc_pose.X;mpc_pose.Y];
    ripb_tmp = ripi_tmp-ribi_tmp;
    rbpb_tmp = Rbi_tmp * ripb_tmp;
%     rbpb_tmp(1) = -p.a/2;
    ripb_tmp = Rbi_tmp' * rbpb_tmp;
    ripi_tmp = ripb_tmp + ribi_tmp;
    
%     ripi_tmp = [T_PS0(1,end);T_PS0(2,end)];
%     
    
    % Get state [x y theta rx ry]
    x = [mpc_pose.X mpc_pose.Y mpc_pose.Theta ripi_tmp'];
    p.x_state_vec = [p.x_state_vec; x];
    p.rpusher_vec = [p.rpusher_vec; rbpb_tmp'];
    
    if abs(T_PS0(2,end)) > 1*(p.b/2)
        disp("ry: ")
        T_PS0(2,end)
    end
    
    % Delay Buffer Simulation
    x_sim = x;
        
    for i=1:1:sample_command_delay_buffer
        u_sim = command_delay_buffer(:,end+1-i);
        Rbi_sim = Helper.C3_2d(x_sim(3));
        vipi_sim = u_sim;
        u_sim = Rbi_sim * u_sim;

        %Transformation of coordinates
        rbbi = Rbi_sim * [x_sim(1); x_sim(2)]; % Rbi * ribi
        rbpi = Rbi_sim * [x_sim(4); x_sim(5)]; % Rbi * ripi

        %Compute derivative (In object frame)
        twist_b = p.dx_funct(rbbi, rbpi, u_sim);
        vbbi = twist_b(1:2);
        dtheta = twist_b(3);
        vibi = Rbi_sim'*vbbi;
        dx = [vibi;dtheta;vipi_sim];% + [normrnd(0,.005);normrnd(0,.005);normrnd(0,.005);0;0]; 
        x_sim = x_sim + p.h_opt*dx';
    end
    
    
    x = x_sim;
    
    % Control MPC
    vipi = p.controller(t.seconds+ p.h_opt*sample_command_delay_buffer, x);
    
    if isinf(p.flag)
        vipi = [0;0];
    end
    
    % Update delay buffer
    if(sample_command_delay_buffer>0)
        command_delay_buffer = [vipi command_delay_buffer(:,1:end-1)];
    end
    
    % Send command to robot
    T_S0B = inv(T_BS0);
    u_robot = T_S0B(1:3,1:3)*[vipi;0];
    
    command_msg.Linear.X = u_robot(1);
    command_msg.Linear.Y = u_robot(2);
    send(command_pub,command_msg)
    
    p.x_state_vec_sim = [p.x_state_vec_sim; x];
    p.u_control_vec = [p.u_control_vec; vipi'];
    p.t_vec = [p.t_vec; t.seconds];
    toc
end

function T = transformToMatrix(transf)
    R = quat2rotm([transf.Transform.Rotation.W, transf.Transform.Rotation.X, transf.Transform.Rotation.Y, transf.Transform.Rotation.Z]);
    P = [transf.Transform.Translation.X, transf.Transform.Translation.Y, transf.Transform.Translation.Z]';
    
    T = [R P; 0 0 0 1];
end



