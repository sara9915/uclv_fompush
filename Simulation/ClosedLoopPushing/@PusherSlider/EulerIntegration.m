%% Euler-Integration
function obj = EulerIntegration(obj, t0, tf, h, d0, qs0, SimulationCounter, filename, tdist, ydist)
    %Set initial conditions
    % i is the inertial reference frame F_a cited in the paper
    ribi0 = [qs0(1); qs0(2)]; % Object coordinate in inertial frame [xo;yo]
    theta0 = qs0(3);
    %Kinematic relations
    % TODO: Change with the helper class function;
    Rbi0 = Helper.C3_2d(theta0); % Rotation matrix from base b to base i
    ripi0 = ribi0 + Rbi0' * [-obj.a / 2; d0]; % Pusher coordinates in inertial frame [xp;yp]
    %Set state variable and initial condition
    number_of_steps = ceil((tf - t0) / h);
    x_state = zeros(number_of_steps, obj.num_states);
    x_state(1,:) = [ribi0' theta0 ripi0']; 
    %Build variables
    t = zeros(number_of_steps,1);
    u_state = zeros(number_of_steps, 2); % TODO Just for me: Generalize for when multiple contact points are included
    delta_u_state = zeros(number_of_steps, 2);
    x_star = zeros(number_of_steps, obj.num_states);
    u_star = zeros(number_of_steps, obj.num_inputs);  
    
    %robot delay
    sample_command_delay_buffer = 5;%3;
    command_delay_buffer = zeros(2,sample_command_delay_buffer);%obj.sample_delay);
    command_delay_buffer(1,:) = obj.u_star(1);
    command_delay_buffer(2,:) = obj.u_star(2);
    
%     obj.x_star_vect(:,end+1) = zeros(1,3);
    
    for step = 1:number_of_steps
        
%         if isinf(obj.flag)
%             t(step + 1) = t(step) + h;
%             obj.t = t;
%             obj.x_state{SimulationCounter+1} = x_state;
%             obj.u_state{SimulationCounter+1} = u_state; 
%             obj.delta_u_state{SimulationCounter} = delta_u_state; 
%             obj.x_state{1} = x_star;  
%             obj.u_state{1} = u_star; 
%             obj.delta_u_pert{1} = obj.delta_u_state{1}(2:end,:) - obj.delta_u_state{1}(1:end-1,:);
%             continue;
%         end
        if step == tdist 
%             disp("Disturbance")
            x_state(step,2) = x_state(step,2) + ydist;
            ribi1 = x_state(step, 1:2)';
            ripi1 = x_state(step, 4:5)';
            ripb1 = ripi1 - ribi1;
            Rbi1 = Helper.C3_2d(x_state(step, 3));
            rbpb1 = Rbi1 * ripb1;
            rbpb2 = [-obj.a / 2; rbpb1(2)];
            ripb2 = Rbi1' * rbpb2;
            ripi2 = ripb2 + ribi1;
            x_state(step,5) = ripi2(2);
        end
%         disp(t(step));
        
        u_star(step,:) = obj.u_star;
%         x_star(step,:) = [obj.u_star(1) * t(step) 0 0 obj.u_star(1) * t(step) - obj.a / 2 0];
        x_star(step,:) = [obj.u_star(1) * t(step) 0 0 obj.u_star(1) * t(step) - obj.a / 2 0];
        
        %Kinematic relations
        Rbi = Helper.C3_2d(x_state(step,3)); %C3_2d(theta)
        %Control input
        
        % ---------------------------
        % simulation response delay
        x_sim =  x_state(step, :);
        
        for i=1:1:obj.sample_delay
            u_sim = command_delay_buffer(:,end+1-i);
            Rbi_sim = Helper.C3_2d(x_sim(3));
            u_sim = Rbi_sim * u_sim;
            vipi_sim = Rbi_sim'*u_sim;
            
            %Transformation of coordinates
            rbbi = Rbi_sim * [x_sim(1); x_sim(2)]; % Rbi * ribi
            rbpi = Rbi_sim * [x_sim(4); x_sim(5)]; % Rbi * ripi

            %Compute derivative (In object frame)
            twist_b = obj.dx_funct(rbbi, rbpi, u_sim);
            vbbi = twist_b(1:2);
            dtheta = twist_b(3);
            vibi = Rbi_sim'*vbbi;
            dx = [vibi;dtheta;vipi_sim]; 
            x_sim = x_sim + h*dx';
            
        end
        
        u_star(step,:) = obj.u_star;
        x_star(step,:) = [obj.u_star(1) * (t(step)+obj.sample_delay*h) 0 0 obj.u_star(1) * (t(step)+obj.sample_delay*h) - obj.a / 2 0];
        % -------------------------
%         if step < obj.sample_delay
%             vipi = obj.controller(0, x_sim); %[0.03;0.03];
%             x_star(step,:) = [obj.u_star(1) * 0 0 0 obj.u_star(1) * 0 - obj.a / 2 0];
%         else
%             vipi = obj.controller(t(step - obj.sample_delay+1)+obj.sample_delay*h, x_sim); %[0.03;0.03];
%             x_star(step,:) = [obj.u_star(1) * (t(step- obj.sample_delay+1)+obj.sample_delay*h) 0 0 obj.u_star(1) * (t(step- obj.sample_delay+1)+obj.sample_delay*h) - obj.a / 2 0];
%         end

        if not(isinf(obj.flag))
%             tic
            vipi = obj.controller(t(step)+obj.sample_delay*h, x_sim); %[0.03;0.03];
%             toc
        else
            vipi = zeros(2,1);
        end
        
        %buffer delay
        if(sample_command_delay_buffer>0)
            command_delay_buffer = [vipi command_delay_buffer(:,1:end-1)];
            vipi = command_delay_buffer(:,end);
        end
        
        %Transformation of coordinates
        rbbi = Rbi * [x_state(step, 1); x_state(step, 2)]; % Rbi * ribi
        rbpi = Rbi * [x_state(step, 4); x_state(step, 5)]; % Rbi * ripi
        vbpi = Rbi * vipi;
        
        u_state(step,1:2) = vbpi;
        delta_u_state(step,1:2) = u_state(step,1:2) - obj.u_star';
        
        %store values
        obj.delay_u_pert = [delta_u_state(step,1:2)' obj.delay_u_pert(:,1:end-1)];
        obj.u_prev_pert = delta_u_state(step,1:2);
        %Compute derivative (In object frame)
        twist_b = obj.dx_funct(rbbi, rbpi, vbpi);
        vbbi = twist_b(1:2);
        dtheta = twist_b(3);
        vibi = Rbi'*vbbi;
        dx = [vibi;dtheta;vipi] + [0;0;0;0; normrnd(0,.0)];%[normrnd(0,.005);normrnd(0,.005);normrnd(0,.005);0;normrnd(0,.01)]; 
        if step < number_of_steps
            t(step + 1) = t(step) + h;
            h_euler = 0.002;
            x_old = x_state(step,:);
            for lv7 = 1:1:ceil(obj.h_opt/h_euler)
                x_new = x_old + h_euler*dx';
                Rbi = Helper.C3_2d(x_new(3)); %C3_2d(theta)
                rbbi = Rbi * [x_new(1); x_new(2)]; % Rbi * ribi
                rbpi = Rbi * [x_new(4); x_new(5)]; % Rbi * ripi
                vbpi = Rbi * vipi;
                twist_b = obj.dx_funct(rbbi, rbpi, vbpi);
                vbbi = twist_b(1:2);
                dtheta = twist_b(3);
                vibi = Rbi'*vbbi;
                dx = [vibi;dtheta;vipi] + [0;0;0;0; normrnd(0,.01)];%[normrnd(0,.005);normrnd(0,.005);normrnd(0,.005);0;normrnd(0,.01)];
                x_old = x_new;
            end
            x_state(step + 1, :) = x_new;
%             if (ripi(1) > 0.0728 && ripi(1) < 0.073)
%                 x_state(i1 + 1, 2:3) = x_state(i1 + 1, 2:3) + [.01 15 * pi / 180];
%             end
            %% Hack to add noise to differential equations
            % It makes sure the pusher is always in contact with the slider
            % TODO: Relook at some point
            % TODO: Consider renaming
            ribi1 = x_state(step + 1, 1:2)';
            ripi1 = x_state(step + 1, 4:5)';
            ripb1 = ripi1 - ribi1;
            Rbi1 = Helper.C3_2d(x_state(step + 1, 3));
            rbpb1 = Rbi1 * ripb1;
            rbpb2 = [-obj.a / 2; rbpb1(2)];
            ripb2 = Rbi1' * rbpb2;
            ripi2 = ripb2 + ribi1;
            x_state(step + 1, 4:5) = ripi2';
            obj.rpusher_vec = [obj.rpusher_vec; rbpb1'];
        end
    end     
    obj.t = t;
    
    obj.x_state{SimulationCounter+1} = x_state;
    obj.u_state{SimulationCounter+1} = u_state; 
    obj.delta_u_state{SimulationCounter} = delta_u_state; 
    obj.x_state{1} = x_star;  
    obj.u_state{1} = u_star; 
    obj.delta_u_pert{1} = obj.delta_u_state{1}(2:end,:) - obj.delta_u_state{1}(1:end-1,:);
    
    params = struct;
    params.x_S = x_state(:,1);
    params.y_S = x_state(:,2);
    params.theta_S = x_state(:,3);
    params.S_p_y = obj.rpusher_vec(:,2);
    params.xref = x_star(:,1);
    params.yref = x_star(:,2);
    params.thetaref = x_star(:,3);
    params.u_n = u_state(:,1);
    params.u_t = u_state(:,2);
    save(strcat("/home/workstation/pusher_slider_matlab/acados_nmpc/statistics_RAL/FOM/",filename),"params");
    
end