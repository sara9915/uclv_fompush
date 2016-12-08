classdef SlidingDownState < HybridState
     properties (Access = protected)
         v0;
     end
    methods
        function obj = SlidingDownState(a, nu_p, c2)
            obj = obj@HybridState(a, nu_p, c2);
            syms u1 u2
            v_MC = [1;obj.gamma_bottom];
            obj.v0 = (u1 / v_MC(1)) * [v_MC(1); v_MC(2)];
            obj = obj.SymbolicLinearize(c2);
        end
    end 
end