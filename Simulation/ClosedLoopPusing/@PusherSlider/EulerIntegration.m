%% Euler-Integration
function obj = EulerIntegration(obj, t0, tf, h, d0, qs0, SimulationCounter)
    %Set initial conditions
    % TODO: Ask Frank what these variables are is r_i i-th position of
    % slider?
    ribi = [qs0(1);qs0(2)];
    theta = qs0(3);
    %Kinematic relations
    Cbi = [cos(theta), sin(theta); -sin(theta), cos(theta)];
    rbpb = Cbi*ribi+[-obj.a/2;d0];
    ripb = Cbi'*rbpb;
    %Set state variable and initial condition
    N = (1/h)*(tf-t0);
    x_state = zeros(N, obj.num_states);
    x_state(1,:) = [ribi' theta ripb']; 
    %Build variables
    t = zeros(N,1);
    u_state = zeros(N, 2); % TODO Just for me: Generalize for when multiple contact points are included
    delta_u_state = zeros(N, 2);
    % TODO: "repmat" can be used, just that everything is cleaner, not really
    % that important
    x_star = zeros(N, obj.num_states);
    u_star = zeros(N, obj.num_inputs);  
    for i1=1:N
        disp(t(i1));
        %Define nominal values
        u_star(i1,:) = obj.u_star;
        x_star(i1,:) = [obj.u_star(1)*t(i1) 0 0 obj.u_star(1)*t(i1)-obj.a/2 0];
        %Object coordinate in inertial frame [xo;yo]
        ribi = [x_state(i1,1);x_state(i1,2)];
        theta = x_state(i1,3);
        %Pusher coordinates in inertial frame [xp;yp]
        ripi = [x_state(i1,4);x_state(i1,5)];
        %Kinematic relations
        Cbi = [cos(theta), sin(theta); -sin(theta), cos(theta)];
        %Control input
        vipi  = obj.controller(t(i1), x_state(i1,:));%[0.03;0.03];%
%         if i1 ==1
%             vipiTemp = vipi;
%         end
        %Transformation of coordinates
        rbbi = Cbi*ribi;
        rbpi = Cbi*ripi;
        vbpi = Cbi*vipi;
        u_state(i1,1:2) = vbpi;
        delta_u_state(i1,1:2) = u_state(i1,1:2) - obj.u_star';
        %Compute derivative (In object frame)
        twist_b = obj.dx_funct(rbbi, rbpi, vbpi);
        vbbi = twist_b(1:2);
        dtheta = twist_b(3);
        vibi = Cbi'*vbbi;
        dx = [vibi;dtheta;vipi];% + [normrnd(0,.005);normrnd(0,.005);normrnd(0,.005);0;0]; 
        if i1 < N
            t(i1 + 1) = t(i1) + h;
            x_state(i1 + 1, :) = x_state(i1, :) + h*dx';
%             if (ripi(1) > 0.0728 && ripi(1) < 0.073)
%                 x_state(i1 + 1, 2:3) = x_state(i1 + 1, 2:3) + [.01 15 * pi / 180];
%             end
            %% Hack to add noise to differential equations
            % It makes sure the pusher is always in contact with the slider
            % TODO: Relook at some point
            % TODO: Consider renaming
            ribi1 = x_state(i1 + 1, 1:2)';
            ripi1 = x_state(i1 + 1, 4:5)';
            ripb1 = ripi1 - ribi1;
            Cbi1 = Helper.C3_2d(x_state(i1 + 1, 3));
            rbpb1 = Cbi1 * ripb1;
            rbpb2 = [-obj.a / 2; rbpb1(2)];
            ripb2 = Cbi1' * rbpb2;
            ripi2 = ripb2 + ribi1;
            x_state(i1 + 1, 4:5) = ripi2';
        end
    end     
    obj.t = t;
    obj.x_state{SimulationCounter+1} = x_state;  
    obj.u_state{SimulationCounter+1} = u_state; 
    obj.delta_u_state{SimulationCounter} = delta_u_state; 
    obj.x_state{1} = x_star;  
    obj.u_state{1} = u_star; 
end