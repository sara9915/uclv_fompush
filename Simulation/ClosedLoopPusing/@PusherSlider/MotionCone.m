function vo = MotionCone(obj,rbbi, rbpi, vbpi)
    %Find rx, ry: In body frame
    rbpb = rbpi - rbbi;
    rx = -obj.a/2;
    ry = rbpb(2);
    %Compute friction cone vectors
    gamma_top    = (obj.nu_p_pert*obj.c_pert^2 - rx*ry + obj.nu_p_pert*rx^2)/(obj.c_pert^2 + ry^2 - obj.nu_p_pert*rx*ry);
    gamma_bottom = (-obj.nu_p_pert*obj.c_pert^2 - rx*ry - obj.nu_p_pert*rx^2)/(obj.c_pert^2 + ry^2 + obj.nu_p_pert*rx*ry);
    try
        gamma = vbpi(2)/vbpi(1);
    catch
        if vbpi(2)>0
            gamma = 1000000;
        else
            gamma = -1000000;
        end
    end        
    %Motion Cone condition for sliding/sticking
    if gamma>gamma_top
        vMC = [1;gamma_top];
        kappa = (vbpi(1))/(vMC(1));
        vo = kappa*vMC;
        disp('Sliding Up');
    elseif gamma<gamma_bottom
        vMC = [1;gamma_bottom];
        kappa = (vbpi(1))/(vMC(1));
        vo = kappa*vMC;
        disp('Sliding Down');
    else
        vo = vbpi;
        disp('Sticking');
    end

    if vo(1)<=0 %|| abs(ry)>obj.b/2
        vo=[0;0];
         disp('not in contact');
    end
end