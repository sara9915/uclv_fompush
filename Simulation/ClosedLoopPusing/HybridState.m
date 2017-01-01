classdef HybridState < matlab.mixin.Heterogeneous
    properties (Access = protected)
        rx;
        rx2;
    end
    properties (Abstract) % Abstract properties cannot define access methods,
       % they have to be defined in the implementation
       controller_matrix; % Kinematics matrix as a linear combination of the controller inputs
       % The linearization of the dynamics of the system allows us to write
       % the friction cone constraints as: E*x_bar + D*u_bar <= g
       name;
       x;
       u;
    end
    
     methods
         function obj = HybridState(a)
             obj.rx = -a / 2.0;
             obj.rx2 = obj.rx * obj.rx;
         end
         % TODO: Change the function so that it returns the required
         % matrices and can be used in more complex trajectories
         % TODO: When generalizing to more complex cases, the computation
         % of all the parameters needs to be done at the implementation
         % classes.
         function [A, B, B_nonlinear, E, D, g, f_star] ...
                   = SymbolicLinearize(obj, x_star, u_star) % x = [x, y, theta, ry] u = [v_n, v_t]
            %Build states
            %Body frame kinematics
            f = obj.controller_matrix * obj.u.';
            %% Linearization
            %Build jacobians
            A = jacobian(f, obj.x);
            B = jacobian(f, obj.u);
            B_nonlinear = matlabFunction(simplify(B));
            % Substitute equilibrium states
            subs_x_u = [x_star, u_star];
            A = double(subs(A, [obj.x, obj.u], subs_x_u));
            B = double(subs(B, [obj.x, obj.u], subs_x_u));
            f_star = matlabFunction(simplify(f)); % TODO: Check with Frank why do we simplify and then cast to something again
            [E, D, g] = obj.GetConstraintMatrices(x_star, u_star);
         end
     end
     methods (Abstract)
         [E, D, g] = GetConstraintMatrices(obj, x_star, u_star);
     end
    
end

