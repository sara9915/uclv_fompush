%% Euler-Integration
function obj = EulerIntegration(obj, t0, tf, h, d0, qs0, SimulationCounter)
    %Set initial conditions
    % i is the inertial reference frame F_a cited in the paper
    ribi0 = [qs0(1); qs0(2)]; % Object coordinate in inertial frame [xo;yo]
    theta0 = qs0(3);
    %Kinematic relations
    % TODO: Change with the helper class function;
    Rbi0 = C3_2d(theta0); % Rotation matrix from base b to base i
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
    for step = 1:number_of_steps
        disp(t(step));
        %Define nominal values
        % TODO: This should be passed as a parameter if we want to set more
        % complex trajectories.
        u_star(step,:) = obj.u_star;
        x_star(step,:) = [obj.u_star(1) * t(step) 0 0 obj.u_star(1) * t(step) - obj.a / 2 0];
        %Kinematic relations
        Rbi = C3_2d(x_state(step,3)); %C3_2d(theta)
        %Control input
        vipi = obj.controller(t(step), x_state(step, :)); %[0.03;0.03];
        %Transformation of coordinates
        rbbi = Rbi * [x_state(step, 1); x_state(step, 2)]; % Rbi * ribi
        rbpi = Rbi * [x_state(step, 4); x_state(step, 5)]; % Rbi * ripi
        vbpi = Rbi * vipi;
        u_state(step,1:2) = vbpi;
        delta_u_state(step,1:2) = u_state(step,1:2) - obj.u_star';
        %Compute derivative (In object frame)
        twist_b = obj.dx_funct(rbbi, rbpi, vbpi);
        vbbi = twist_b(1:2);
        dtheta = twist_b(3);
        vibi = Rbi'*vbbi;
        dx = [vibi;dtheta;vipi];% + [normrnd(0,.005);normrnd(0,.005);normrnd(0,.005);0;0]; 
        if step < number_of_steps
            t(step + 1) = t(step) + h;
            x_state(step + 1, :) = x_state(step, :) + h*dx';
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
        end
    end     
    obj.t = t;
    obj.x_state{SimulationCounter+1} = x_state;  
    obj.u_state{SimulationCounter+1} = u_state; 
    obj.delta_u_state{SimulationCounter} = delta_u_state; 
    obj.x_state{1} = x_star;  
    obj.u_state{1} = u_star; 
end