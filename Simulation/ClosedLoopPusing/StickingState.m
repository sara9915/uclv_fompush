classdef StickingState < HybridState
properties
    E;
    D;
    g;
end
properties (Access = protected)
    controller_matrix;
end
methods
    function obj = StickingState(a, nu_p, c2)
        obj = obj@HybridState(a, nu_p, c2);
        syms ry
        Q = [c2 + obj.rx2 obj.rx * ry;
             obj.rx * ry c2 + ry^2] ./ (c2 + obj.rx2 + ry^2);
        P = eye(2); % Not really needed, but keeps consistency
        b = [-ry / (c2 + obj.rx2 + ry^2) obj.rx];
        c = [0 0];
        %Build nonlinear function
        obj.controller_matrix = [obj.Cbi.' * Q * P; b; c];
        obj = obj.SymbolicLinearize();
        obj.E = obj.v_n_star * [-obj.C_top_linear; obj.C_bottom_linear];
        obj.D = [-obj.gamma_top_star 1; obj.gamma_bottom_star -1];
        obj.g = [-obj.v_t_star + obj.gamma_top_star * obj.v_n_star;
                 obj.v_t_star - obj.gamma_bottom_star * obj.v_n_star];
    end
end 
end

