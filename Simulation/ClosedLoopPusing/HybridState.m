classdef HybridState
    properties (Access = private)
        % Declare Equilibrium variables
        u_star = [0.05;0];
        ry_star = 0;
        x_eq = [0;0;0];
        %Kinematics
        Cbi;
        rx;
    end
    properties (Access = protected)
        gamma_top;
        gamma_bottom;
    end
    properties (Abstract, Access = protected)
       v0;
    end
    properties
        C_top_linear;
        C_bottom_linear;
        gamma_top_star;
        gamma_bottom_star;
        A_linear;
        B_linear;
        B_nonlinear; % TODO: Change names
        f_star;
    end
    
     methods
         function obj = HybridState(a, nu_p, c2)
             syms theta ry
             obj.rx = -a/2.0;
             rx2 = obj.rx * obj.rx;
             obj.Cbi = Helper.C3_2d(theta);
             obj.gamma_top = (nu_p * c2 - obj.rx * ry + nu_p * rx2) / (c2 + ry^2 - nu_p * obj.rx * ry);
             obj.gamma_bottom = (-nu_p * c2 - obj.rx * ry - nu_p * rx2) / (c2 + ry^2 + nu_p * obj.rx * ry);
         end
         function obj = SymbolicLinearize(obj, c2) 
            syms x y theta
            syms ry u1 u2
            %Build states
            v0x = obj.v0(1);
            v0y = obj.v0(2);
            x_state = [x;y;theta;ry];
            u_state = [u1;u2];
            rx2 = obj.rx * obj.rx;
            rbpb = [obj.rx;ry];
            %Define gamma=vt/vn
            C_top = jacobian(obj.gamma_top,x_state);
            obj.C_top_linear = double(subs(C_top, ry, {obj.ry_star}));
            C_bottom = jacobian(obj.gamma_bottom, x_state);
            obj.C_bottom_linear = double(subs(C_bottom, ry, {obj.ry_star}));
            obj.gamma_top_star = double(subs(obj.gamma_top, ry, obj.ry_star));
            obj.gamma_bottom_star = double(subs(obj.gamma_bottom, ry, obj.ry_star));
            
            %Body frame kinematics
            dx_b = ((c2 + rx2) * v0x + obj.rx * ry * v0y) / (c2 + rx2 + ry^2);
            dy_b = ((c2 + ry^2) * v0y + obj.rx * ry * v0x) / (c2 + rx2 + ry^2);
            dtheta = (obj.rx * dy_b - ry * dx_b) / c2;
            %Kinematics
            drbbi = [dx_b;dy_b];
            dribi = transpose(obj.Cbi) * drbbi;
            drbpb = [u1;u2] - [dx_b;dy_b] - Helper.cross3d(dtheta, rbpb);
            dry = simplify(drbpb(2));%[u1;u2] - obj.v0;%
            %Build nonlinear function
            f = [dribi;dtheta;dry];

            %Build jacobians
            A = jacobian(f,x_state);
            B = jacobian(f,u_state);
            obj.B_nonlinear = matlabFunction(simplify(B));
            % Substitute equilibrium states
            A = subs(A,{x,y,theta ry},{obj.x_eq(1),obj.x_eq(2),obj.x_eq(3), obj.ry_star});
            A = subs(A,{u1,u2},{obj.u_star(1),obj.u_star(2)});
            B = subs(B,{x,y,theta ry},{obj.x_eq(1),obj.x_eq(2),obj.x_eq(3), obj.ry_star});
            B = subs(B,{u1,u2},{obj.u_star(1),obj.u_star(2)});
            %Convert to double type
            A = double(A);
            B = double(B);
    %         Compute LQR solution
    %         K = lqr(A,B,obj.Q_LQR,obj.R_LQR);
    %         obj.K = K;  
            %Set properties
            obj.A_linear = double(A);
            obj.B_linear = double(B);
            obj.f_star = matlabFunction(simplify(f)); % TODO: Check with Frank whether this makes sense
         end
     end
    
end

