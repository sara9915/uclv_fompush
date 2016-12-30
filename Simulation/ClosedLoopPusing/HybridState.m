classdef HybridState < matlab.mixin.Heterogeneous
    properties (Access = private)
        % Declare Equilibrium variables
        ry_star = 0;
        x_eq = [0;0;0];
    end
    properties (Access = protected)
        rx;
        rx2;
        Cbi;
        gamma_top;
        gamma_bottom;
        v_n_star;
        v_t_star;
        gamma_top_star; % Part of the c.c. linearization params
        gamma_bottom_star; % Part of the c.c. linearization params
        C_top_linear; % Part of the cone constraint(c.c) linearization params
        C_bottom_linear; % Part of the c.c. linearization params
    end
    properties (Abstract, Access = protected)
       controller_matrix; % Kinematics matrix as a linear combination of the controller inputs
    end
    properties (Abstract)
       % The linearization of the dynamics of the system allows us to write
       % the friction cone constraints as: E*x_bar + D*u_bar <= g
       E;
       D;
       g; 
    end
    properties
       A_linear;
       B_linear;
       B_nonlinear; % TODO: Change names
       f_star;
       u_star = [0.05;0]; % TODO: Change to allow complex trajectories
    end
    
     methods
         function obj = HybridState(a, nu_p, c2)
             syms theta ry
             obj.rx = -a/2.0;
             obj.rx2 = obj.rx * obj.rx;
             obj.Cbi = Helper.C3_2d(theta);
             obj.gamma_top = (nu_p * c2 - obj.rx * ry + nu_p * obj.rx2) / (c2 + ry^2 - nu_p * obj.rx * ry);
             obj.gamma_bottom = (-nu_p * c2 - obj.rx * ry - nu_p * obj.rx2) / (c2 + ry^2 + nu_p * obj.rx * ry);
         end
         function obj = SymbolicLinearize(obj)
            syms x y theta
            syms ry v_n v_t % ry is py in the paper TODO: Change
            %Build states
            x_state = [x; y; theta; ry];
            u_state = [v_n; v_t];
            %Body frame kinematics
            f = obj.controller_matrix * u_state;

            %% Linearization
            obj.v_n_star = obj.u_star(1);
            obj.v_t_star = obj.u_star(2);
            C_top = jacobian(obj.gamma_top,x_state);
            C_bottom = jacobian(obj.gamma_bottom, x_state);
            %Build jacobians
            A = jacobian(f, x_state);
            B = jacobian(f, u_state);
            obj.B_nonlinear = matlabFunction(simplify(B));
            % Substitute equilibrium states
            A = subs(A,{x,y,theta ry},{obj.x_eq(1),obj.x_eq(2),obj.x_eq(3), obj.ry_star});
            A = subs(A,{v_n,v_t},{obj.v_n_star, obj.v_t_star});
            B = subs(B,{x,y,theta ry},{obj.x_eq(1),obj.x_eq(2),obj.x_eq(3), obj.ry_star});
            B = subs(B,{v_n,v_t},{obj.v_n_star, obj.v_t_star});
    %         Compute LQR solution
    %         K = lqr(A,B,obj.Q_LQR,obj.R_LQR);
    %         obj.K = K;  
            %Set properties
            obj.A_linear = double(A);
            obj.B_linear = double(B);
            obj.C_top_linear = double(subs(C_top, ry, {obj.ry_star}));
            obj.C_bottom_linear = double(subs(C_bottom, ry, {obj.ry_star}));
            obj.gamma_top_star = double(subs(obj.gamma_top, ry, obj.ry_star));
            obj.gamma_bottom_star = double(subs(obj.gamma_bottom, ry, obj.ry_star));
            obj.f_star = matlabFunction(simplify(f)); % TODO: Check with Frank whether this makes sense
         end
     end
    
end

