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
% 
%                 disp('x_MPC');
%                 out_delta_x{Family}(:,1)'
%                 disp('u_MPC');
%                 disp(out_delta_u{Family}(1,1:2)')
            catch
                fval{Family}=nan;
                out_delta_u{Family} = zeros(obj.steps,2);
%                 disp('Opt. not feasible');
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
%         SolutionVec
        
        % FILTER
%         num_samples = 24;
%         if(t>num_samples*0.025)
%             coeff = ones(1,num_samples)/num_samples;
%             avg = filter(coeff,1,obj.solutionVec);
%             SolutionVec = avg(end,:);
%         end


        % 
        if(numel(obj.index)~=0)
            factor_pref = (1e-3*PusherSlider.u_star(1))/(5e-3);%0.004;%1.4e-3; %1e-3; %0.001 for nu_p = 0.19
%             SolutionVec = SolutionVec + [-factor_pref 0 0];
            SolutionVec(obj.index(end)) =  SolutionVec(obj.index(end))- factor_pref;
            obj.solutionVec = [obj.solutionVec; SolutionVec];
        end
        [minFval, index] = min(SolutionVec);
%     
%          %Soluzione temporanea
%        if (norm(minFval-SolutionVec(1)) < 1e-1)
%             index = 1;
%             minFval = SolutionVec(index);
%        end

        obj.index = [obj.index; index];
        
        delta_u = out_delta_u{index}(1,1:2)'; 
        
        obj.deltau_st = [obj.deltau_st; out_delta_u{1}(1,1:2)]; 
        obj.deltau_su = [obj.deltau_su; out_delta_u{2}(1,1:2)]; 
        obj.deltau_sd = [obj.deltau_sd; out_delta_u{3}(1,1:2)]; 
        obj.deltau = [obj.deltau; out_delta_u{index}(1,1:2)];
         % u filtering
%         obj.u_old = [obj.u_old,delta_u];
%         num_samples = 3;
%         if(t>num_samples*0.025)
%             coeff = ones(1,num_samples)/num_samples;
%             avg = filter(coeff,1,obj.u_old);
%             delta_u = avg(:,end);
%         end
        
        obj.CostVector = [obj.CostVector;minFval];

        if index==1
            obj.modes=[obj.modes;0];
%             disp('Sticking Controller');
%             disp(delta_u)
        elseif index==2
            obj.modes=[obj.modes;1];
%             disp('Sliding Up Controller');
%             disp(delta_u)
        else
            obj.modes=[obj.modes;-1];
%             disp('Sliding Down Controller');
%             disp(delta_u)
        end
%         figure(2000), hold on
%         figure(2000), plot(t,delta_u(1),'*')
%         figure(2001), hold on
%         figure(2001), plot(t,delta_u(2),'*')
% %         obj.delta_u_prev = delta_u;
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
%         disp('start')
        [obj.Opt.MIQP, solvertime, fval] = obj.Opt.MIQP.solve; % TODO: Two variables unused
%         disp('stop')
        out = obj.Opt.MIQP.vars.u.value(1:2)';
        obj.Opt.MIQP.vars.region.value;
        delta_u = out(1:2);
%         out_delta_x = obj.Opt.MIQP.vars.x.value'; % Used to plot
%         out_delta_u = obj.Opt.MIQP.vars.u.value'; % Used to plot
%         obj.PlotMPC(out_delta_u,out_delta_x);
%         return
    end
    % Control u to applied velocity (world frame)
%     disp("-------------")
%     obj.flag 
%     obj.traj_x0
%     if obj.flag == 3
%         dummy = 0;
%     end
    Rx0_0 = rotz(obj.traj_x0(3));
   
    Cbi = Helper.C3_2d(theta);
%     vbpi = (obj.delta_u_delay + obj.u_star);

    vbpi = (delta_u + obj.u_star);
    
%     vipi = Rx0_0(1:2,1:2)*Cbi'*vbpi;
    vipi = Cbi'*vbpi;
    

    
end