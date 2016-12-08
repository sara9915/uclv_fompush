classdef Friction < dynamicprops
    properties (Constant)
      g = 9.81;
      n1 = [1;0;1];
      n2 = [0;1;0];
      n3 = [0;0;1];
    end
    
    properties      
        p;
    end
    
    methods
        %% Constructor
        function obj = Friction(p)
            obj.p = p;

        end
       
        
    end %End dynamic methods
    methods(Static)
             
        function cross = cross( x)
            cross = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
        end
       
        %% Rotation Matrices
        function C1 = C1(x)
            C1 = [1 0 0; 0 cos(x) sin(x); 0 -sin(x) cos(x)];
        end
        
        function C2 = C2(x)
            C2 = [cos(x) 0 -sin(x); 0 1 0; sin(x) 0 cos(x)];
        end
        
        function C3 = C3(x)
            C3 = [ cos(x) sin(x) 0; -sin(x) cos(x) 0; 0 0 1 ];
        end
        %% Numerical Integration
        function integral = DoubleGaussQuad(fun1,a,b,c,d)
            %Change of Variables
            h1 = (b-a)/2;
            h2 = (b+a)/2;
            h3 = (d-c)/2;
            h4 = (d+c)/2;

            %Define Weights (for 3 points)
            w1 = 1;
            w2 = 1;

            %Define Points
            x1 = sqrt(1/3);
            x2 = -sqrt(1/3);

            integral = h1 * h3 * (w1*w1*fun1(h1*x1+h2, h1*x1+h2) + w1*w2*fun1(h1*x1+h2, h1*x2+h2) +...
                w2*w1*fun1(h1*x2+h2, h1*x1+h2) + w2*w2*fun1(h1*x2+h2, h1*x2+h2) );
        end

    end % end methods (Static)

end