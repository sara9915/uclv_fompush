function twist = dx_funct(obj,rbbi, rbpi, vbpi)
    %Find rx, ry: In body frame
    rbpb = rbpi - rbbi;
    rx = rbpb(1);
    ry = rbpb(2);
    %Determine 1) If Sticking 2) Sliding velocity
    vp = MotionCone(obj,rbbi, rbpi, vbpi);
    vpx = vp(1);
    vpy = vp(2);
    %Compute derivative: In body frame
    vx = ((obj.c_pert^2 + rx^2)*vpx + rx*ry*vpy)/(obj.c_pert^2 + rx^2 + ry^2);
    vy = ((obj.c_pert^2 + ry^2)*vpy + rx*ry*vpx)/(obj.c_pert^2 + rx^2 + ry^2);
    dtheta = (rx*vy - ry*vx)/(obj.c_pert^2); 
    %Build output vector
    twist = [vx;vy;dtheta];
end