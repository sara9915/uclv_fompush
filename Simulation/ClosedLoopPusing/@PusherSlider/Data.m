        %% Animation Data
        function AniOutput = Data(obj, ite, Dataset)  
            x_state = obj.x_state{Dataset};
            u_state = obj.u_state{Dataset};
            %Declare variables
            x     = x_state(ite, 1);
            y     = x_state(ite, 2);
            theta = x_state(ite, 3);
            xp     = x_state(ite, 4);
            yp     = x_state(ite, 5);
            u1 = u_state(ite, 1);
            u2 = u_state(ite, 2);
            %Find distance d
            Cbi = Helper.C3_2d(theta);
            ribi = [x;y];
            ripi = [xp;yp];
            ripb = ripi-ribi;
            rbpb = Cbi*ripb;
            d = rbpb(2);
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
            %%
            rbpb2 = [-obj.a/2; rbpb(2)];
            ripb2 = R*rbpb2;
            ripi2 = ripb2 + ribi;
            
            %%
            pos = ripi2;
            pos2 = pos + R*[-Radius;0];
            numPoints=100; %Number of points making up the circle
            %Define circle in polar coordinates (angle and radius)
            theta_vec=linspace(0,2*pi,numPoints); %100 evenly spaced points between 0 and 2pi
            rho=ones(1,numPoints)*Radius; %Radius should be 1 for all 100 points
            %Convert polar coordinates to Cartesian for plotting
            [X,Y] = pol2cart(theta_vec,rho); 
            X = X+pos2(1);
            Y = Y+pos2(2);
            %Compute gammas
            rx = -obj.a/2;
            gamma_top     = (obj.nu_p*obj.c^2 - rx*d + obj.nu_p*rx^2)/(obj.c^2 + d^2 - obj.nu_p*rx*d);
            gamma_bottom = (-obj.nu_p*obj.c^2 - rx*d - obj.nu_p*rx^2)/(obj.c^2 + d^2 + obj.nu_p*rx*d);
            gamma_top_linear    = obj.gammaTop_star    + obj.C_top_linear(4)*d;%(obj.nu_p*obj.c^2 - rx*d + obj.nu_p*rx^2)/(obj.c^2 + d^2 - obj.nu_p*rx*d);
            gamma_bottom_linear = obj.gammaBottom_star + obj.C_bottom_linear(4)*d;%(-obj.nu_p*obj.c^2 - rx*d - obj.nu_p*rx^2)/(obj.c^2 + d^2 + obj.nu_p*rx*d);
            %Compute vectors
            vTop_b = [1;gamma_top];
            vBottom_b = [1;gamma_bottom];
            vTop_linear_b = [1;gamma_top_linear];
            vBottom_linear_b = [1;gamma_bottom_linear];
            vTop_i = Cbi'*vTop_b;
            vBottom_i = Cbi'*vBottom_b;
            vTop_linear_i = Cbi'*vTop_linear_b;
            vBottom_linear_i = Cbi'*vBottom_linear_b;
            %Compute velocity vectors
            vbpi = [u1;u2];
            vipi = Cbi'*vbpi;

            %Set ouputs
            AniOutput.x1rot = x1rot; 
            AniOutput.y1rot = y1rot; 
            AniOutput.pos = pos;
            AniOutput.X = X;
            AniOutput.Y = Y;
            AniOutput.vTop_b = vTop_b;
            AniOutput.vBottom_b = vBottom_b;
            AniOutput.vTop_linear_b = vTop_linear_b;
            AniOutput.vBottom_linear_b = vBottom_linear_b;
            AniOutput.d = d;
            AniOutput.vipi = vipi;
        end