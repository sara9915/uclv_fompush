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