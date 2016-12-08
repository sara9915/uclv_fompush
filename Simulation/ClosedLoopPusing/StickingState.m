classdef StickingState < HybridState
     properties (Access = protected)
         v0;
     end
    methods
        function obj = StickingState()
            obj = obj@HybridState(a, nu_p, c2);
            syms u1 u2
            obj.v0 = [u1;u2];
            obj = obj.SymbolicLinearize(c2);
        end
    end 
end

