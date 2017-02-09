%% Symbolic Manipulation
function obj = symbolicNonLinearize(obj)
    %Symbolic variables
    syms x y theta a xp yp
    syms ry  u1 u2
    %Build states                        
    x_state = [x;y;theta;ry];
    u_state = [u1;u2];
    %Kinematics
    Cbi = Helper.C3_2d(theta);
    rx = -obj.a/2;
    rbpb = [rx;ry];
    
    %% Stick: vo = vp, Slide Up: vo = vo_up, Slide Down: vo = vo_down
    %Define gamma=vt/vn
    gamma_top    = (obj.nu_p*obj.c^2 - rx*ry + obj.nu_p*rx^2)/(obj.c^2 + ry^2 - obj.nu_p*rx*ry);
    gamma_bottom = (-obj.nu_p*obj.c^2 - rx*ry - obj.nu_p*rx^2)/(obj.c^2 + ry^2 + obj.nu_p*rx*ry);

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
        B{lv1} = jacobian(f{lv1},u_state);
        obj.B_Nonlinear{lv1} = matlabFunction(simplify(B{lv1}));
        obj.f_star{lv1} = matlabFunction(simplify(f{lv1}));
    end
end