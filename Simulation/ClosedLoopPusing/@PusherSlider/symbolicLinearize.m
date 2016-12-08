%% Symbolic Manipulation
function obj = symbolicLinearize(obj)
    %Symbolic variables
    syms x y theta a xp yp
    syms ry  u1 u2
    %Build states                        
    x_state = [x;y;theta;ry];
    u_state = [u1;u2];
    % Declare Equilibrium variables
    obj.u_star= [0.05;0];
    obj.ry_star= [0];
    obj.x_eq= [0;0;0];
    %Kinematics
    Cbi = Helper.C3_2d(theta);
    rx = -obj.a/2;
    rbpb = [rx;ry];
    
    %% Stick: vo = vp, Slide Up: vo = vo_up, Slide Down: vo = vo_down
    %Define gamma=vt/vn
    gamma_top    = (obj.nu_p*obj.c^2 - rx*ry + obj.nu_p*rx^2)/(obj.c^2 + ry^2 - obj.nu_p*rx*ry);
    gamma_bottom = (-obj.nu_p*obj.c^2 - rx*ry - obj.nu_p*rx^2)/(obj.c^2 + ry^2 + obj.nu_p*rx*ry);
    C_top = jacobian(gamma_top,x_state);
    C_top = subs(C_top, ry, {obj.ry_star});
    C_top = double(C_top);
    C_bottom = jacobian(gamma_bottom,x_state);
    C_bottom = subs(C_bottom, ry, {obj.ry_star});
    C_bottom = double(C_bottom);
    obj.C_top_linear = double(C_top);
    obj.C_bottom_linear = double(C_bottom);
    obj.gammaTop_star    =    double(subs(gamma_top, ry, obj.ry_star));
    obj.gammaBottom_star = double(subs(gamma_bottom, ry, obj.ry_star));
    
    %Determine effective pusher velocity equation
    for lv1=1:3
        if lv1 == 1;%Sticking
            %Define motion cone boundary vector
            vo = [u1;u2];
            vox = u1;
            voy = u2;
        elseif lv1==2;%Sliding up
            v_MC = [1;gamma_top];
            vo = (u1/v_MC(1))*v_MC; 
            vox = vo(1);
            voy = vo(2);
        else
            v_MC = [1;gamma_bottom];
            vo = (u1/v_MC(1))*[v_MC(1);v_MC(2)];
            vox = vo(1);
            voy = vo(2);
        end
        %Body frame kinematics
        dx_b{lv1} = ((obj.c^2+rx^2)*vox + rx*ry*voy)/(obj.c^2+rx^2+ry^2);
        dy_b{lv1} = ((obj.c^2+ry^2)*voy + rx*ry*vox)/(obj.c^2+rx^2+ry^2);
        dtheta{lv1} = 1/obj.c^2*(rx*dy_b{lv1} - ry*dx_b{lv1});
        %Kinematics
        drbbi{lv1} = [dx_b{lv1};dy_b{lv1}];
        dribi{lv1} = transpose(Cbi)*drbbi{lv1};
        drbpb{lv1} = [u1;u2] - [dx_b{lv1};dy_b{lv1}] - Helper.cross3d([dtheta{lv1}], [rbpb]);
        dry{lv1}   = simplify(drbpb{lv1}(2))  ;%[u1;u2] - vo;%
        %Build nonlinear function
        f{lv1} = [dribi{lv1};dtheta{lv1};dry{lv1}];

        %Build jacobians
        A{lv1} = jacobian(f{lv1},x_state);
        B{lv1} = jacobian(f{lv1},u_state);
        % Substitute equilibrium states
        A{lv1} = subs(A{lv1},{x,y,theta ry},{obj.x_eq(1),obj.x_eq(2),obj.x_eq(3), obj.ry_star});
        A{lv1} = subs(A{lv1},{u1,u2},{obj.u_star(1),obj.u_star(2)});
        B{lv1} = subs(B{lv1},{x,y,theta ry},{obj.x_eq(1),obj.x_eq(2),obj.x_eq(3), obj.ry_star});
        B{lv1} = subs(B{lv1},{u1,u2},{obj.u_star(1),obj.u_star(2)});
        %Convert to double type
        A{lv1}=double(A{lv1});
        B{lv1}=double(B{lv1});
%         Compute LQR solution
%         K = lqr(A,B,obj.Q_LQR,obj.R_LQR);
%         obj.K = K;  
        %Set properties
        obj.A_linear{lv1} = double(A{lv1});
        obj.B_linear{lv1} = double(B{lv1});
    end
end