classdef SlidingUpState < HybridState
properties
    E;
    D;
    g;
end
properties (Access = protected)
    controller_matrix;
end
properties (Access = private)
    epsilon = 0.005;
end
methods
    function obj = SlidingUpState(a, nu_p, c2)
        obj = obj@HybridState(a, nu_p, c2);
        syms ry
        Q = [c2 + obj.rx2 obj.rx * ry;
             obj.rx * ry c2 + ry^2] ./ (c2 + obj.rx2 + ry^2);
        P = [1 0; obj.gamma_top 0];
        b = [(-ry + obj.gamma_top * obj.rx) / (c2 + obj.rx2 + ry^2) 0];
        c = [-obj.gamma_top 0];
        %Build nonlinear function
        obj.controller_matrix = [obj.Cbi.' * Q * P; b; c];
        obj = obj.SymbolicLinearize();
        obj.E = obj.v_n_star * obj.C_top_linear;
        obj.D = [obj.gamma_top_star -1];
        obj.g = [obj.v_t_star - obj.gamma_top_star * obj.v_n_star - obj.epsilon];
    end
end 
end