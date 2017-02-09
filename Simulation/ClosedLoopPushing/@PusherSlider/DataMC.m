        %% Animation Data
        function AniOutput = DataMC(obj, x_state)        
            %Declare variables
            x     = x_state(1);
            y     = x_state( 2);
            theta = x_state(3);
            xp     = x_state( 4);
            yp     = x_state(5);
           %Rotation matrix
            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            Radius = 0.0045;
            % Vertices of Polygon
            P1 = [x; y] + R*[-obj.a/2; -obj.b/2];
            P2 = [x; y] + R*[obj.a/2; -obj.b/2];
            P3 = [x; y] + R*[obj.a/2; obj.b/2];
            P4 = [x; y] + R*[-obj.a/2; obj.b/2];
            %Build vector of vertices
            x1rot = [P1(1) P2(1) P3(1) P4(1)];
            y1rot = [P1(2) P2(2) P3(2) P4(2)];
            pos = [xp;yp];
            pos2 = [xp;yp] + R*[-Radius;0];
            numPoints=100; %Number of points making up the circle
            %Define circle in polar coordinates (angle and radius)
            theta_vec=linspace(0,2*pi,numPoints); %100 evenly spaced points between 0 and 2pi
            rho=ones(1,numPoints)*Radius; %Radius should be 1 for all 100 points
            %Convert polar coordinates to Cartesian for plotting
            [X,Y] = pol2cart(theta_vec,rho); 
            X = X+pos2(1);
            Y = Y+pos2(2);
            %Set ouputs
            AniOutput.x1rot = x1rot; 
            AniOutput.y1rot = y1rot; 
            AniOutput.pos = pos;
            AniOutput.X = X;
            AniOutput.Y = Y;
        end