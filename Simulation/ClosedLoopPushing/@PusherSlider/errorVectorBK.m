%%Controller Design (In world frame)
function delta_x = errorVector(obj, t, x_state)   
    x = x_state(1);
    y = x_state(2);
    theta = x_state(3);
    xp = x_state(4);
    yp = x_state(5);
    %Kinematics
    Rbi = Helper.C3_2d(theta);
    ripi = [xp;yp];
    ribi = [x;y];
    ripb = ripi-ribi;
    rbpb = Rbi * ripb;
    d = rbpb(2);
    %Compute x_star
    if strcmp(obj.ControllerType,'Trajectory')
        x_star = [obj.u_star(1)*t; 0; 0];
        %Compute error vector
        delta_d = d - obj.ry_star;
        delta_x = [x_state(1:3)' - x_star; delta_d];
    elseif strcmp(obj.ControllerType,'Target')
        %% Set Target Position
        if obj.flag == 0
%             riti = [0.23;-0.11];
            riti = [0.30;0.0];
            if norm([riti(1) - x; riti(2) - (y)])<0.01
                obj.flag = 1;
            end
%         elseif obj.flag == 1
% %             riti = [0.23;0.11];
%             riti = [0.20;0.0];
%             if norm([riti(1) - x; riti(2) - (y)])<0.01
%                 obj.flag = 2;
%             end
%         elseif obj.flag == 2
% %             riti = [-0.05;0.05];
%               riti = [0.30;0];
%               if norm([riti(1) - x; riti(2) - (y)])<0.01
%                 obj.flag = 3;
%               end
        else
            riti = [0;0];
        end
        %%
        obj.flag
        ritb = riti - ribi;
        rbtb = Rbi*ritb;
        theta_rel = -imag(log(rbtb(1) + 1i*rbtb(2)));
        theta_g = theta - theta_rel;
        Cci = Helper.C3_2d(theta_g);
        vcbi = Cci*ribi; % NON UTILIZZATA
        delta_d = d - obj.ry_star;
        delta_x = [0;0;theta_rel;delta_d];
    else
        disp('Error: Could not find proper flag string');
    end
    
end