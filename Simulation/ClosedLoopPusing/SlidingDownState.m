classdef SlidingDownState < HybridState
properties
    controller_matrix;
    name = 'Sliding down';
    x;
    u;
    gamma_bottom;
    C_bottom;
end
properties (Access = private)
    epsilon = 0.005;
end
methods
    function obj = SlidingDownState(a, nu_p, c2)
        obj = obj@HybridState(a);
        obj.x = sym('x', [1, 4]); % x, y, theta, ry
        obj.u = sym('u', [1, 2]); % v_n, v_t
        obj.gamma_bottom = (-nu_p * c2 - obj.rx * obj.x(4) - nu_p * obj.rx2) / (c2 + obj.x(4)^2 + nu_p * obj.rx * obj.x(4));
        Cbi = Helper.C3_2d(obj.x(3));
        Q = [c2 + obj.rx2 obj.rx * obj.x(4);
             obj.rx * obj.x(4) c2 + obj.x(4)^2] ./ (c2 + obj.rx2 + obj.x(4)^2);
        P = [1 0; obj.gamma_bottom 0];
        b = [(-obj.x(4) + obj.gamma_bottom * obj.rx) / (c2 + obj.rx2 + obj.x(4)^2) 0];
        c = [-obj.gamma_bottom 0];
        %Build nonlinear function
        obj.controller_matrix = [Cbi.' * Q * P; b; c];
        obj.C_bottom = jacobian(obj.gamma_bottom, obj.x);
    end
    function [E, D, g] = GetConstraintMatrices(obj, x_star, u_star)
        C_bottom_subs = double(subs(obj.C_bottom, obj.x(4), x_star(4)));
        gamma_bottom_star = double(subs(obj.gamma_bottom, obj.x(4), x_star(4)));
        E = -u_star(1) * C_bottom_subs;
        D = [-gamma_bottom_star 1];
        g = [-u_star(2) + gamma_bottom_star * u_star(1) - obj.epsilon];
    end
end 
end