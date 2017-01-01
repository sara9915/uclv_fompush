%%Controller Design (In world frame)
function vipi = controller(obj, t, x_state)   
    ribi = [x_state(1);x_state(2)]; % [x;y]
    theta = x_state(3);
    ripi = [x_state(4);x_state(5)]; % [xp;yp]
    %Kinematics
    Cbi = Helper.C3_2d(theta);
    ripb = ripi-ribi;
    rbpb = Cbi*ripb;
    d = rbpb(2); % TODO: Never used
    %Compute error vector
    delta_x = obj.errorVector(t, x_state);%[obj.u_star(1)*t; 0; 0];
    % MPC Control 
    if obj.FOM
        options = optimoptions('quadprog','Display','none'); %TODO: Never used
        % TODO: this same loop was already used before, more reasons for
        % the class/object approach
        % TODO: In this situation, counter = Family, one of the two
        % variables is useless
        for Family=1:obj.NumFam
            if Family ==1
                counter = 1;
            elseif Family ==2
                counter = 2;
            else
                counter = 3; 
            end 
            obj.UpdateMatricesIC(counter, delta_x, Family);
            try
                [obj.Opt.FOM{Family}, solvertime{Family}, fval{Family}] = obj.Opt.FOM{Family}.solve;
                out_delta_u{Family} = obj.Opt.FOM{Family}.vars.u.value';
                out_delta_x{Family} = obj.Opt.FOM{Family}.vars.x.value';

%                 disp('x_MPC');
%                 out_delta_x{Family}(:,1)'
%                 disp('u_MPC');
%                 out_delta_u{Family}(1,1:2)'
            catch
                fval{Family}=100000000;
                disp('Opt. not feasible');
            end
            %Reinitialize matrices
            obj.Opt.FOM{Family}.b = obj.binInitial{Family};
            obj.Opt.FOM{Family}.A = obj.AinInitial{Family};
            obj.Opt.FOM{Family}.beq = obj.beqInitial{Family};
            obj.Opt.FOM{Family}.Aeq = obj.AeqInitial{Family};
             
        end
        %Find best solution within FOM
        if obj.NumFam==1
            SolutionVec = [fval{1}];
        elseif obj.NumFam==3
            SolutionVec = [fval{1} fval{2} fval{3}];
        else
            SolutionVec = [fval{1} fval{2} fval{3} fval{4} fval{5} fval{6} fval{7} fval{8} fval{9}];
        end
        [minFval, index] = min(SolutionVec);
        
        delta_u = out_delta_u{index}(1,1:2)'; 
        obj.CostVector = [obj.CostVector;minFval];

        if index==1
            obj.modes=[obj.modes;0];
            disp('Sticking Controller');
        elseif index==2
            obj.modes=[obj.modes;1];
            disp('Sliding Up Controller');
        else
            obj.modes=[obj.modes;-1];
            disp('Sliding Down Controller');
        end
%         obj.delta_u_prev = delta_u;
%         obj.delta_u_delay = Helper.smooth(obj, delta_u, 0.2, obj.delta_u_delay);
%         for lv2=1:obj.NumFam
%             obj.PlotMPC(out_delta_u{lv2},out_delta_x{lv2});
%         end
% return
    else
        %Set initial conditions
        Ain1 = zeros(obj.num_vars, obj.Opt.MIQP.nv); % TODO: Not used
        %Build Initial condition b matrix
%         binTemp = [obj.A_bar{1}*delta_x; -obj.A_bar{1}*delta_x;obj.A_bar{2}*delta_x; -obj.A_bar{2}*delta_x;obj.A_bar{3}*delta_x; -obj.A_bar{3}*delta_x];
        binTemp = [obj.A_bar{1}*delta_x; -obj.A_bar{1}*delta_x;obj.A_bar{2}*delta_x; -obj.A_bar{2}*delta_x;obj.A_bar{3}*delta_x; -obj.A_bar{3}*delta_x];
        bin = obj.binInitial + [binTemp; zeros(length(obj.binInitial)-length(binTemp),1)];
        obj.Opt.MIQP.b = bin;  
        disp('start')
        [obj.Opt.MIQP, solvertime, fval] = obj.Opt.MIQP.solve; % TODO: Two variables unused
        disp('stop')
        out = obj.Opt.MIQP.vars.u.value(1:2)';
        obj.Opt.MIQP.vars.region.value;
        delta_u = out(1:2);
%         out_delta_x = obj.Opt.MIQP.vars.x.value'; % Used to plot
%         out_delta_u = obj.Opt.MIQP.vars.u.value'; % Used to plot
%         obj.PlotMPC(out_delta_u,out_delta_x);
%         return
    end
    % Control u to applied velocity (world frame)
    Cbi = Helper.C3_2d(theta);
%     vbpi = (obj.delta_u_delay + obj.u_star);
    vbpi = (delta_u + obj.u_star);
    vipi = Cbi'*vbpi;
end