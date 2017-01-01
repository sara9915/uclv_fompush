classdef HybridMode
%MODE Class that defines a sequence of HybridStates and allows the
% construction of the optimization program using FOM
properties
    optimization_program;
end
properties (Access = private)
    hybrid_states;
    number_of_variables;
    number_of_controllers;
end

methods
    function [obj] = HybridMode(hybrid_states, number_of_variables, number_of_controllers)
        if nargin > 0
            obj.hybrid_states = hybrid_states;
            obj.number_of_variables = number_of_variables;
            obj.number_of_controllers = number_of_controllers;
        end
    end
    function [optimization_problem] = GetOptimizationProblem(obj, h_opt, a, Q_MPC, Q_MPC_final, R_MPC, hybrid_states_map)
        number_of_states = length(obj.hybrid_states); % Number of steps in the optimization program
        % Define discrete linear dynamic matrices
        % Define optimization program
        optimization_problem = MixedIntegerConvexProgram(false);
        % TODO: Check next two lines
        optimization_problem = optimization_problem.addVariable('u', 'C', [obj.number_of_controllers, number_of_states], ...
            -100 * ones(obj.number_of_controllers,number_of_states), 100*ones(obj.number_of_controllers,number_of_states));
        optimization_problem = optimization_problem.addVariable('x', 'C', [obj.number_of_variables, number_of_states], ...
            -100 * ones(obj.number_of_variables, number_of_states), 100 * ones(obj.number_of_variables, number_of_states));

        % Loop through steps of opt. program
        for state_index = 1:number_of_states;
            hybrid_state = hybrid_states_map(obj.hybrid_states(state_index)); % Todo change if it slows everything down
            %% Cost
            H = zeros(optimization_problem.nv, optimization_problem.nv);
            c = zeros(optimization_problem.nv, 1);
            % Assign cost block matrices
            if state_index < number_of_states
                H(optimization_problem.vars.x.i(1:length(Q_MPC),state_index), optimization_problem.vars.x.i(1:length(Q_MPC),state_index)) = zeros(length(Q_MPC));
                H(optimization_problem.vars.x.i(1:length(Q_MPC),state_index), optimization_problem.vars.x.i(1:length(Q_MPC),state_index)) = Q_MPC;
            end
            H(optimization_problem.vars.u.i(1:length(R_MPC),state_index), optimization_problem.vars.u.i(1:length(R_MPC),state_index)) = R_MPC;
            %Final Cost
            if state_index == number_of_states
                H(optimization_problem.vars.x.i(1:length(Q_MPC_final),state_index), optimization_problem.vars.x.i(1:length(Q_MPC_final),state_index)) = Q_MPC_final;
            end
            optimization_problem = optimization_problem.addCost(H, [], []); 
            if state_index == 1
                %% Dynamic Constraints
                % TODO: This seems off. Why nothing happens at initial step?
%                      Ain_motion = zeros(obj.num_vars, optimization_problem.nv);
%                  Ain1(:,optimization_problem.vars.x.i(1:obj.num_vars,state_index)) = zeros(obj.num_vars);
%                  bin1 = zeros(obj.num_vars,1);
%                  Ain2 = -Ain1;
%                  bin2 = bin1;
%                  optimization_problem = optimization_problem.addLinearConstraints(Ain1, bin1, [], []);
%                  optimization_problem = optimization_problem.addLinearConstraints(Ain2, bin2, [], []);
%                  clear Ain1 Ain2 bin1 bin2
                %% Cone Constraints
%                if counter==1
%                    Ain = zeros(2, optimization_problem.nv);
%                    bin = [0;0];
%                    optimization_problem = optimization_problem.addLinearConstraints(Ain, bin, [], []);  
%                elseif counter==2
%                    Ain = zeros(2, optimization_problem.nv);
%                    bin=[0;0];
%                    optimization_problem = optimization_problem.addLinearConstraints(Ain, bin, [], []);   
%                else
%                    Ain = zeros(2, optimization_problem.nv);
%                    bin=[0;0];
%                    optimization_problem = optimization_problem.addLinearConstraints(Ain, bin, [], []);   
%                end
%                clear Ain bin
            else
                %% Dynamic Constraints
                A_bar = eye(length(hybrid_state.A_linear)) + h_opt * hybrid_state.A_linear;
                B_bar = h_opt * hybrid_state.B_linear;
                assert(size(A_bar, 1) == size(B_bar, 1), 'A_bar row number: %d and B_bar row number: %d mismatch', size(A_bar, 1), size(B_bar, 1));
                assert(size(A_bar, 1) == size(A_bar, 2), 'A_bar row number: %d and A_bar column number: %d mismatch', size(A_bar, 1), size(A_bar, 2));
                assert(size(A_bar, 1) == obj.number_of_variables, 'A_bar row number: %d and number of variables: %d mismatch', size(A_bar, 1), obj.number_of_variables);
                assert(size(B_bar, 2) == obj.number_of_controllers, 'B_bar column number: %d and number of controllers: %d mismatch', size(B_bar, 2), obj.number_of_controllers);
                %% Motion Cone Constraint
                D = hybrid_state.D;
                E = hybrid_state.E;
                g = hybrid_state.g;
                assert(size(E, 1) == size(D, 1), 'E row number: %d, and D row number: %d mismatch', size(E, 1), size(D, 1));
                assert(size(D, 1) == size(g, 1), 'D row number: %d, and g row number: %d mismatch', size(D, 1), size(g, 1));
                assert(size(E, 2) == size(A_bar, 2), 'E column number: %d, and A_bar column number: %d mismatch', size(E, 2), size(A_bar, 2));
                assert(size(D, 2) == size(B_bar, 2), 'D column number: %d, and B_bar column number: %d mismatch', size(D, 2), size(B_bar, 2));
                optimization_problem.nv
                Ain_motion = zeros(obj.number_of_variables, optimization_problem.nv);
                Ain_motion(:,optimization_problem.vars.x.i(1:obj.number_of_variables, state_index-1)) = -A_bar;
                Ain_motion(:,optimization_problem.vars.x.i(1:obj.number_of_variables, state_index)) = eye(obj.number_of_variables);
                Ain_motion(:,optimization_problem.vars.u.i(1:obj.number_of_controllers, state_index)) = -B_bar;
                bin_motion = 0;
                number_of_motion_cone_constraints = size(D,1);
                Ain_cone = zeros(number_of_motion_cone_constraints, optimization_problem.nv);
                bin_cone = g;
                Ain_cone(:, optimization_problem.vars.x.i(1:obj.number_of_variables, state_index)) = E;
                Ain_cone(:, optimization_problem.vars.u.i(1:obj.number_of_controllers, state_index)) = D;
                optimization_problem = optimization_problem.addLinearConstraints(...
                    [Ain_motion; -Ain_motion; Ain_cone], ...
                    [bin_motion; bin_motion; bin_cone], [], []);   
            end
            % TODO: Encapsulate somehow in a function or something, it
            % cannot be floating around here.
            %% Vn min
            Ain_vn_min = zeros(1, optimization_problem.nv);
            Ain_vn_min(:, optimization_problem.vars.u.i(1,state_index)) = -1;
            bin_vn_min = hybrid_state.u_star(1) - 0.01;
            %Target:-0.03
            %Traj:-0.01
            %% Vn max
            Ain_vn_max = zeros(1, optimization_problem.nv);
            Ain_vn_max(:,optimization_problem.vars.u.i(1,state_index)) = 1;
            bin_vn_max = -hybrid_state.u_star(1) + 0.1;
            %% Vt min
            Ain_vt_min = zeros(1, optimization_problem.nv);
            Ain_vt_min(:, optimization_problem.vars.u.i(2, state_index)) = -1;
%             bin = obj.u_star(2) + 0.1;
            bin_vt_min = hybrid_state.u_star(2) + 0.1;
            %% Vt max
            Ain_vt_max = zeros(1, optimization_problem.nv);
            Ain_vt_max(:, optimization_problem.vars.u.i(2, state_index)) = 1;
%             bin = -obj.u_star(2) + 0.1;
            bin_vt_max = -hybrid_state.u_star(2) + 0.1;
            %% Max py distance
            Ain_py_max = zeros(1, optimization_problem.nv);
            Ain_py_max(:, optimization_problem.vars.x.i(4, state_index)) = 1;
            bin_py_max = .75 * a / 2;
            %% Min py distance
            Ain_py_min = zeros(1, optimization_problem.nv);
            Ain_py_min(:, optimization_problem.vars.x.i(4, state_index)) = -1;
            bin_py_min = .75 * a / 2;
            optimization_problem = optimization_problem.addLinearConstraints( ...
            [Ain_vn_min; Ain_vn_max; Ain_vt_min; Ain_vt_max; 
             Ain_py_max; Ain_py_min], ...
            [bin_vn_min; bin_vn_max; bin_vt_min; bin_vt_max;
             bin_py_max; bin_py_min], [], []);
        end
        % Define struct to store initial value for bin (before IC added at each time step)
        % TODO: Check if required to store them
%         obj.AinInitial = optimization_problem.A;
%         obj.binInitial = optimization_problem.b;
%         obj.AeqInitial = optimization_problem.Aeq;
%         obj.beqInitial = optimization_problem.beq;
        %Save data 
        obj.optimization_program = optimization_problem;
    end
    function vipi = controller(obj, t, x_state)   
        ribi = [x_state(1); x_state(2)]; % [x;y]
        theta = x_state(3);
        ripi = [x_state(4); x_state(5)]; % [xp;yp]
        %Kinematics
        Cbi = Helper.C3_2d(theta);
        ripb = ripi - ribi;
        rbpb = Cbi * ripb;
        d = rbpb(2); % TODO: Never used
        %Compute error vector
        delta_x = obj.errorVector(t, x_state);%[obj.u_star(1)*t; 0; 0];
        % MPC Control 
        if obj.FOM
            options = optimoptions('quadprog','Display','none'); %TODO: Never used
            for Family = 1:obj.NumFam
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

            if index == 1
                obj.modes = [obj.modes; 0];
                disp('Sticking Controller');
            elseif index == 2
                obj.modes=[obj.modes; 1];
                disp('Sliding Up Controller');
            else
                obj.modes = [obj.modes; -1];
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
            binTemp = [obj.A_bar{1} * delta_x; -obj.A_bar{1} * delta_x; obj.A_bar{2} * delta_x; -obj.A_bar{2} * delta_x; obj.A_bar{3} * delta_x; -obj.A_bar{3} * delta_x];
            bin = obj.binInitial + [binTemp; zeros(length(obj.binInitial) - length(binTemp), 1)];
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
        vipi = Cbi' * vbpi;
    end
end
    
end

