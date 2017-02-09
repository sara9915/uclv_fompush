classdef FOMSolver < MPCSolvers.MPCSolver
%FOMSOLVER Implementation of the MPCSolver that solves it by using the
%Family of Modes (FOM) approach.

properties (Access = private)
    hybrid_modes; % Array with the hybrid_modes to consider for the FOM.
    chameleon_mode;
    has_chameleon;
end

methods
function obj = FOMSolver(hybrid_states_map, Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt, hybrid_modes, has_chameleon)
    obj = obj@MPCSolvers.MPCSolver(hybrid_states_map, Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt);
    assert(~isempty(hybrid_modes) > 0, 'Empty hybrid_modes array');
    obj.hybrid_modes = hybrid_modes;
    obj.has_chameleon = has_chameleon;
    obj.chameleon_mode = [];
end

function [f_values] = GetDataFirstStep(obj, current_t, u_star, current_x)
    number_of_modes = size(obj.hybrid_modes, 1);
    out_u_bar = cell(1, number_of_modes);
    f_values = Inf * ones(1, number_of_modes);
    for hybrid_mode_index = 1:number_of_modes
        hybrid_mode = current_hybrid_modes(hybrid_mode_index, :);
        optimization_problem = obj.GetOptimizationProblem(current_t, x_star, u_star, current_x, hybrid_mode);
        try
            [solved_optimization_problem, solvertime, f_values(hybrid_mode_index)] = optimization_problem.solve;
            out_u_bar{hybrid_mode_index} = solved_optimization_problem.vars.u.value;
        catch
            f_values(hybrid_mode_index) = Inf; % Disregard this solution
            fprintf('Opt. number %d not feasible\n', hybrid_mode_index);
        end
    end
    [min_cost, mode_index] = min(f_values);
    try
        u_bar = out_u_bar{mode_index}(1:obj.number_of_controllers, 1);
    catch
        error('Could not find a solution for any optimization problem');
    end
end

function [u_state, mode_index, min_cost, obj] = SolveMPC(obj, current_t, x_star, u_star, current_x)
    options = optimoptions('quadprog','Display','none'); %TODO: Never used
    current_hybrid_modes = obj.hybrid_modes;
    if obj.has_chameleon
        current_hybrid_modes = [current_hybrid_modes; obj.chameleon_mode];
    end
    number_of_modes = size(current_hybrid_modes, 1);
    out_u_bar = cell(1, number_of_modes);
%     out_x_bar = cell(1, number_of_modes);
    f_values = Inf * ones(1, number_of_modes);
    for hybrid_mode_index = 1:number_of_modes
        hybrid_mode = current_hybrid_modes(hybrid_mode_index, :);
        optimization_problem = obj.GetOptimizationProblem(current_t, x_star, u_star, current_x, hybrid_mode);
        try
            [solved_optimization_problem, solvertime, f_values(hybrid_mode_index)] = optimization_problem.solve;
            out_u_bar{hybrid_mode_index} = solved_optimization_problem.vars.u.value;
        catch
            f_values(hybrid_mode_index) = Inf; % Disregard this solution
            fprintf('Opt. number %d not feasible\n', hybrid_mode_index);
        end
    end
    [min_cost, mode_index] = min(f_values);
    try
        u_bar = out_u_bar{mode_index}(1:obj.number_of_controllers, 1);
    catch
        error('Could not find a solution for any optimization problem');
    end
    disp([sprintf('Mode %d. First state: ', mode_index), obj.hybrid_states_map(current_hybrid_modes(mode_index, 1)).name]);
    u_state = u_bar + u_star(current_t);
    if obj.has_chameleon
        obj.chameleon_mode = [current_hybrid_modes(mode_index, 2:end), current_hybrid_modes(mode_index, 1)];
    end
end
end
methods
% methods (Access = private)
function [optimization_problem] = GetOptimizationProblem(obj, t0, x_star, u_star, x0, hybrid_mode)
    number_of_steps = length(hybrid_mode); % Number of hybrid states in the hybrid_mode, it corresponds to the number of steps of the MPC problem
    t = t0:obj.h_opt:(t0 + obj.h_opt * (number_of_steps - 1));
    u_lb = -u_star(t) - obj.u_lower_bound * ones(1, number_of_steps);
    u_ub = -u_star(t) + obj.u_upper_bound * ones(1, number_of_steps);
    x_lb = - obj.x_lower_bound * ones(1, number_of_steps);
    x_ub = obj.x_upper_bound * ones(1, number_of_steps);
    optimization_problem = MixedIntegerConvexProgram(false); % Define optimization program
    % The arguments for the function are (name, type_, size_, lower_bound, upper_bound, start_value)
    optimization_problem = optimization_problem.addVariable('x', 'C', [obj.number_of_variables, number_of_steps], x_lb, x_ub);
    optimization_problem = optimization_problem.addVariable('u', 'C', [obj.number_of_controllers, number_of_steps], u_lb, u_ub);
    % Loop through steps of opt. program
    for step_index = 1:number_of_steps;
        hybrid_state_index = hybrid_mode(step_index);
        hybrid_state = obj.hybrid_states_map(hybrid_state_index); % Todo change if it slows everything down
        %% Cost
        H = zeros(optimization_problem.nv, optimization_problem.nv);
        H(optimization_problem.vars.x.i(1:length(obj.Q_MPC), step_index), optimization_problem.vars.x.i(1:length(obj.Q_MPC), step_index)) = obj.Q_MPC;
        H(optimization_problem.vars.u.i(1:length(obj.R_MPC), step_index), optimization_problem.vars.u.i(1:length(obj.R_MPC), step_index)) = obj.R_MPC;
        %Final Cost
        if step_index == number_of_steps
            H(optimization_problem.vars.x.i(1:length(obj.Q_MPC_final), step_index), optimization_problem.vars.x.i(1:length(obj.Q_MPC_final), step_index)) = obj.Q_MPC + obj.Q_MPC_final;
        end
        optimization_problem = optimization_problem.addCost(H, [], []);
        A_motion = zeros(obj.number_of_variables, optimization_problem.nv);
        A_motion(:,optimization_problem.vars.x.i(1:obj.number_of_variables, step_index)) = eye(obj.number_of_variables);
        % TODO: Should add constraint to bound the first value to it's
        % real value?
        if step_index == 1
            delta_x0 = x0 - x_star(t(1));
            [B, F, D, g] = hybrid_state.GetInitialStateMatrices(x0, x_star(t(1)), u_star(t(1)));
            %% Add nonlinear dynamic constraint
            F_bar = delta_x0 + obj.h_opt * F;
            B_bar = obj.h_opt * B;
            assert(size(B_bar, 1) == obj.number_of_variables, 'B_bar row number: %d and number of variables: %d mismatch', size(B_bar, 1), obj.number_of_variables);
            assert(size(B_bar, 2) == obj.number_of_controllers, 'B_bar column number: %d and number of controllers: %d mismatch', size(B_bar, 2), obj.number_of_controllers);
            assert(size(D, 1) == size(g, 1), 'D row number: %d, and g row number: %d mismatch', size(D, 1), size(g, 1));
            assert(size(D, 2) == size(B_bar, 2), 'D column number: %d, and B_bar column number: %d mismatch', size(D, 2), size(B_bar, 2));
            %Add constraint (Modify existing dummy constraint)
            A_motion(:,optimization_problem.vars.u.i(1:obj.number_of_controllers, 1)) = -B_bar;
            b_motion = F_bar;
            number_of_motion_cone_constraints = size(D,1);
            A_constraint = zeros(number_of_motion_cone_constraints, optimization_problem.nv);
        else
            [A, B, D, E, g] = hybrid_state.GetLinearMatrices(x_star(t(step_index)), u_star(t(step_index)));
            %% Dynamic Constraints
            A_bar = eye(size(A)) + obj.h_opt * A;
            B_bar = obj.h_opt * B;
            assert(size(A_bar, 1) == size(B_bar, 1), 'A_bar row number: %d and B_bar row number: %d mismatch', size(A_bar, 1), size(B_bar, 1));
            assert(size(A_bar, 1) == size(A_bar, 2), 'A_bar row number: %d and A_bar column number: %d mismatch', size(A_bar, 1), size(A_bar, 2));
            assert(size(A_bar, 1) == obj.number_of_variables, 'A_bar row number: %d and number of variables: %d mismatch', size(A_bar, 1), obj.number_of_variables);
            assert(size(B_bar, 2) == obj.number_of_controllers, 'B_bar column number: %d and number of controllers: %d mismatch', size(B_bar, 2), obj.number_of_controllers);
            assert(size(E, 1) == size(D, 1), 'E row number: %d, and D row number: %d mismatch', size(E, 1), size(D, 1));
            assert(size(D, 1) == size(g, 1), 'D row number: %d, and g row number: %d mismatch', size(D, 1), size(g, 1));
            assert(size(E, 2) == size(A_bar, 2), 'E column number: %d, and A_bar column number: %d mismatch', size(E, 2), size(A_bar, 2));
            assert(size(D, 2) == size(B_bar, 2), 'D column number: %d, and B_bar column number: %d mismatch', size(D, 2), size(B_bar, 2));
            A_motion(:,optimization_problem.vars.x.i(1:obj.number_of_variables, step_index-1)) = -A_bar;
            A_motion(:,optimization_problem.vars.u.i(1:obj.number_of_controllers, step_index)) = -B_bar;
            b_motion = zeros(size(A_motion, 1),1);
            number_of_motion_cone_constraints = size(D,1);
            A_constraint = zeros(number_of_motion_cone_constraints, optimization_problem.nv);
            A_constraint(:, optimization_problem.vars.x.i(1:obj.number_of_variables, step_index)) = E;
        end
        A_constraint(:, optimization_problem.vars.u.i(1:obj.number_of_controllers, step_index)) = D;
        b_constraint = g;
        optimization_problem = optimization_problem.addLinearConstraints(A_constraint, b_constraint, A_motion, b_motion);
    end
end

end
    
end

