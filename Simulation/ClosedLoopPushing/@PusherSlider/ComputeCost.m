function obj = ComputeCost(obj, d, x_state, lv1, lv2, lv3)
    %Compute nominal state
    x_star = [obj.u_star(1)*0; 0; 0];
    x = x_state(1);
    y = x_state(2);
    theta = x_state(3);
    %Kinematics
    delta_d = d - obj.ry_star;
    delta_x = [x_state(1:3) - x_star; delta_d];
    % LQR Control
%             delta_u = -obj.K*delta_x;
    % MPC Control 
    options = optimoptions('quadprog','Display','none');
    for Family=1:obj.NumFam
        if Family ==1
            counter = 1;
        elseif Family ==2
            counter = 2;
        else
            counter = 3;
        end
        %Build Initial condition b matrix
        binTemp = [obj.A_bar{counter}*delta_x; -obj.A_bar{counter}*delta_x];
        %Solve optimization program
        bin{Family} = obj.binInitial{Family} + [binTemp; zeros(length(obj.binInitial{Family})-length(binTemp),1)];
        obj.Opt.FOM{Family}.b = bin{Family};  
        [obj.Opt.FOM{Family}, solvertime{Family}, fval{Family}] = obj.Opt.FOM{Family}.solve;
         out_delta_u{Family} = obj.Opt.FOM{Family}.vars.u.value';
         out_delta_x{Family} = obj.Opt.FOM{Family}.vars.x.value';
    end
    %Find best solution within FOM
%     if obj.NumFam==3
%         SolutionVec = [fval{1} fval{2} fval{3}];
%     else
%         SolutionVec = [fval{1} fval{2} fval{3} fval{4} fval{5} fval{6} fval{7} fval{8} fval{9}];
%     end
%     [minFval index] = min(SolutionVec);

%Store cost
obj.StateCost{lv1, lv2, lv3} = [x,y,theta];
obj.Cost{lv1, lv2, lv3} = [fval{1} fval{2} fval{3}];

end