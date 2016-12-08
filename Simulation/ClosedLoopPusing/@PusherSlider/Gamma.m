function obj = Gamma(obj,ry)

    rx = -obj.a/2;
    %Compute friction cone vectors
    gamma_top    = (obj.nu_p_pert*obj.c_pert^2 - rx*ry + obj.nu_p_pert*rx^2)/(obj.c_pert^2 + ry^2 - obj.nu_p_pert*rx*ry);
    gamma_bottom = (-obj.nu_p_pert*obj.c_pert^2 - rx*ry - obj.nu_p_pert*rx^2)/(obj.c_pert^2 + ry^2 + obj.nu_p_pert*rx*ry);
  
    
end