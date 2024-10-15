%%Controller Design (In world frame)
function delta_x = errorVector(obj, t, x_state)

    if(isinf(obj.flag))
        delta_x = zeros(4,1);
    return
    end
    if obj.flag == 0
        obj.traj_x0 = [0 0 0]';%x_state(1:3)';
        obj.flag = obj.flag+1;
    end

    xold = x_state(1:3);
    Txstate_0 = pose2T(x_state(1:3));
    Tx0_0 = pose2T(obj.traj_x0');
    Txstate_x0 = Tx0_0\Txstate_0;
    x_state(1:2) = Txstate_x0(1:2,3);
    x_state(3) = atan2(Txstate_x0(2,1),Txstate_x0(1,1));
%     Rz = rotz(rad2deg(obj.traj_x0(3)));
%     x_state(1:2) = (Rz(1:2,1:2)' * x_state(1:2)' + obj.traj_x0(1:2))';
%     x_state(3) = (x_state(3) - obj.traj_x0(3)); 
    
   
    pusher_tmp = Tx0_0\[x_state(4:5) 1]';
    x_state(4:5) = pusher_tmp(1:2);
    
    
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
        points = [
%             0.06 0
%             0.12 0.10
              % EXPERIMENT 1
%               0.30 0
              % EXPERIMENT 3
              0.07 0 
              0.12 0.10
              0.16 0.15 
%                0.10 0 
%             [0.12 0.10]+2*([0.12 0.10]-[0.08 0])
%             [0.08 0]+2*([0.12 0.10]-[0.08 0])
            ];
        
        
        % SI assume che la pura rotazione sia sempre in flag pari
        x_finals = gen_ref(points);
        
          
        Txflag_0 = pose2T(x_finals(obj.flag,1:3));     
        Txflag_x0 = Tx0_0\Txflag_0;
        x_finals(obj.flag,1:2) = Txflag_x0(1:2,3);
        x_finals(obj.flag,3) = atan2(Txflag_x0(2,1),Txflag_x0(1,1));
        
        
        delta_d = d - obj.ry_star;
        
        x_des = x_finals(obj.flag,:)';
        
%         t;
%         obj.flag
        norm(x_state(3)-x_des(3));
%         norm(x_state(1:2)' - x_des(1:2))
        
        x_star = zeros(3,1);
        
        [x_star(1:2), end_time_lin] = lin_traj(t, obj.t0, [0;0], x_des(1:2), obj.lin_vel_star);
        
        [x_star(3), end_time_rot] = lin_traj(t, obj.t0, 0, x_des(3), obj.rot_vel_star);
        

        if mod(obj.flag,2) == 0 % solo rotazione
            switch_cond = norm(x_state(3)-x_des(3)) < deg2rad(1);
        else 
            switch_cond = norm(x_state(1:2)' - x_des(1:2))<0.01;
        end
            norm(x_state(1:2)' - x_des(1:2));
        if end_time_lin &&  end_time_rot  
            if obj.flag >= size(x_finals,1)
                delta_x = zeros(4,1);
                obj.flag = Inf;
%                 disp("END")
                return
            end
            obj.flag = obj.flag+1;
            obj.t0 = t;
            %x_des = x_finals(obj.flag,:)';
%             obj.traj_x0 = xold';
%             obj.traj_x0(1:2) = xold(1:2);
            x_finals = gen_ref(points);
            obj.traj_x0(1:2) = x_finals(obj.flag-1,1:2)';
            obj.traj_x0(3) = x_finals(obj.flag-1,3)';
            delta_x = errorVector(obj, t, x_state);
            return;
        end
        
%         x0 = zeros(3,1);
%         if obj.flag>0
%             x0 = x_finals(obj.flag,:)';
%         end
        
        Tstar_x0 = pose2T(x_star');     
        Tstar_0 = Tx0_0*Tstar_x0;
        obj.x_star_vect(:,end+1) = [Tstar_0(1:2,3)' atan2(Tstar_0(2,1),Tstar_0(1,1))];
        
        delta_x = [ x_state(1:3)' - x_star; delta_d ];
        return
        
        
        %%
%         obj.flag
%         ritb = riti - ribi;
%         rbtb = Rbi*ritb;
%         theta_rel = -imag(log(rbtb(1) + 1i*rbtb(2)));
%         theta_g = theta - theta_rel;
%         Cci = Helper.C3_2d(theta_g);
%         vcbi = Cci*ribi; % NON UTILIZZATA
%         delta_d = d - obj.ry_star;
%         delta_x = [0;0;theta_rel;delta_d];
    else
%         disp('Error: Could not find proper flag string');
    end
    
end


function [p,end_time] = lin_traj(t, t0, p0, pf, vel)
    end_time = false;
    if t <= t0
        p = p0;
        return;
    end
    
    dur = norm(pf-p0)/vel;
    tf = t0+dur;
    
    if t >= tf
        p = pf;
        end_time = true;
        return;
    end
    
    p = p0 + unit(pf-p0)*vel*(t-t0);
    

end

function y = unit(x)
    if norm(x) < eps
       y = zeros(size(x));
       y(1) = 1;
       warning("unit zero vect")
       return;
    end
    y = x/norm(x);
end

function x_finals = gen_ref(points)
    % points = [x1 y1; x2 y2; ... ; xN yN];
    % x_finals = [x1 y1 th1; x2 y2 th2; ... ; xN yN thN];
    
    x_finals = zeros(2*size(points,1)-1,3);
    
%     OLD VERSION
%     x_finals = zeros(length(points),3);
%     x_finals(:,1:2) = points;
%     x_finals(1,3) = atan2(points(1,2), points(1,1));
%     x_finals(2:end,3) = atan2(points(2:end,2) ,points(2:end,1) - points(1:end-1,1));
    j = 1;
%      NEW VERSION
    for i = 1:size(x_finals,1)   
       x_finals(i,1:2) = points(j,:);
       if mod(i,2) == 0 && j < size(points,1)
%            x_finals(i,3) = -x_finals(i-1,3) + atan2(points(j+1,2)-points(j,2), points(j+1,1)-points(j,1));
            x_finals(i,3) = atan2(points(j+1,2)-points(j,2), points(j+1,1)-points(j,1));
           j = j+1;
       else
           if i>1 
              x_finals(i,3) = x_finals(i-1,3); 
           end 
       end 
    end
    
%     x_finals = x_finals(1:2:end,:);
%     x_x_finals = linspace(0.05,0.25,50);
%         y_x_finals = linspace(0,0.10,50);
%         theta_x_finals = linspace(0,0.5880,50);
%         x_finals = [x_x_finals' y_x_finals' theta_x_finals'];

end

function T = pose2T(x)
    % x = [x y theta]
    Rz = rotz(x(3));
    T = [Rz(1:2,1:2) x(1:2)'; 0 0 1];
end

