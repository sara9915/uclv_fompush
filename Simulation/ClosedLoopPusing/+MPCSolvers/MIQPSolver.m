classdef MIQPSolver < MPCSolvers.MPCSolver
%MIQPSOLVER Implementation of the MPCSolver that solves it by using the
%Mixed Integer Quadratic Programming (MIQP) approach.

properties
    number_of_steps; % Number of steps in the optimization program
end

properties (Access = private)
    number_of_states;
    clustering_factor;
end

methods
function obj = MIQPSolver(hybrid_states_map, Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt, number_of_steps, clustering_factor)
    obj = obj@MPCSolvers.MPCSolver(hybrid_states_map, Q_MPC, Q_MPC_final, R_MPC, u_lower_bound, u_upper_bound, x_lower_bound, x_upper_bound, h_opt);
    obj.number_of_steps = number_of_steps;
    obj.number_of_states = length(obj.hybrid_states_map);
    obj.clustering_factor = clustering_factor;
end
    
function [u_state, state_index, min_cost, obj] = SolveMPC(obj, current_t, x_star, u_star, current_x)
    options = optimoptions('quadprog','Display','none'); %TODO: Never used
    optimization_problem = obj.GetOptimizationProblem(current_t, x_star, u_star, current_x);
    [solved_optimization_problem, solvertime, min_cost] = optimization_problem.solve;
%     out_x_bar = solved_optimization_problem.vars.x.value;
    out_u_bar = solved_optimization_problem.vars.u.value;
    out_z = solved_optimization_problem.vars.z.value;
    u_bar = out_u_bar(:, 1);
    state_index = (1:obj.number_of_states) * out_z(:, 1);
    u_state = u_bar + u_star(current_t);
%     out_x_bar = solved_optimization_problem.vars.x.value'; % Used to plot
%     out_u_bar = solved_optimization_problem.vars.u.value'; % Used to plot
%     obj.PlotMPC(out_u_bar, out_x_bar);
%     return
end
end
methods
% methods(Access = private)
function [optimization_problem] = GetOptimizationProblem(obj, t0, x_star, u_star, x0)
    M = 100;
    %Define discrete linear dynamic matrices
    t = t0:obj.h_opt:(t0 + obj.h_opt * (obj.number_of_steps - 1));
    u_lb = -u_star(t) - obj.u_lower_bound * ones(1, obj.number_of_steps);
    u_ub = -u_star(t) + obj.u_upper_bound * ones(1, obj.number_of_steps);
    x_lb = -obj.x_lower_bound * ones(1, obj.number_of_steps);
    x_ub = obj.x_upper_bound * ones(1, obj.number_of_steps);
    %Define optimization program
    optimization_problem = MixedIntegerConvexProgram(false);
    optimization_problem = optimization_problem.addVariable('x', 'C', [obj.number_of_variables, obj.number_of_steps], x_lb, x_ub);
    optimization_problem = optimization_problem.addVariableIfNotPresent('u', 'C', [obj.number_of_controllers, obj.number_of_steps], u_lb, u_ub);
    number_of_clusters = ceil(obj.number_of_steps ./ obj.clustering_factor);
    optimization_problem = optimization_problem.addVariableIfNotPresent('z', 'B', [obj.number_of_states, number_of_clusters], 0, 1);
    %Loop through steps of opt. program
    for step_index = 1:obj.number_of_steps;
        %% Define Cost Functions
        H = zeros(optimization_problem.nv, optimization_problem.nv);
        H(optimization_problem.vars.x.i(1:length(obj.Q_MPC), step_index), optimization_problem.vars.x.i(1:length(obj.Q_MPC), step_index)) = obj.Q_MPC;
        H(optimization_problem.vars.u.i(1:length(obj.R_MPC), step_index), optimization_problem.vars.u.i(1:length(obj.R_MPC), step_index)) = obj.R_MPC;
        %Final Cost
        if step_index == obj.number_of_steps
            H(optimization_problem.vars.x.i(1:length(obj.Q_MPC_final), step_index), optimization_problem.vars.x.i(1:length(obj.Q_MPC_final), step_index)) = obj.Q_MPC + obj.Q_MPC_final;
        end
        optimization_problem = optimization_problem.addCost(H, [], []); 

        current_x_lb = x_lb(:, step_index);
        current_x_ub = x_ub(:, step_index);
        current_u_lb = u_lb(:, step_index);
        current_u_ub = u_ub(:, step_index);
        %% Define mode dependant constraints
        for state_index = 1:obj.number_of_states
            A_motion = zeros(obj.number_of_variables, optimization_problem.nv);
            A_motion(:,optimization_problem.vars.x.i(:, step_index)) = eye(obj.number_of_variables);
            hybrid_state = obj.hybrid_states_map(state_index);
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
                %Add constraint
                A_motion(:, optimization_problem.vars.u.i(:, step_index)) = -B_bar;
                b_motion = F_bar;
                number_of_motion_cone_constraints = size(D,1);
                A_constraint = zeros(number_of_motion_cone_constraints, optimization_problem.nv);
                [motion_matrix_lb, motion_matrix_ub] = obj.GetBoundsMatrixTimesVector(B_bar, current_u_lb, current_u_ub);
                [~, constraint_matrix_ub] = obj.GetBoundsMatrixTimesVector(D, current_u_lb, current_u_ub);
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
                A_motion(:, optimization_problem.vars.x.i(:, step_index - 1)) = -A_bar;
                A_motion(:, optimization_problem.vars.u.i(:, step_index)) = -B_bar;
                b_motion = 0 * ones(obj.number_of_variables, 1);
                [A_bar_lb, A_bar_ub] = obj.GetBoundsMatrixTimesVector(A_bar, current_x_lb, current_x_ub);
                [B_bar_lb, B_bar_ub] = obj.GetBoundsMatrixTimesVector(B_bar, current_u_lb, current_u_ub);
                motion_matrix_lb = A_bar_lb + B_bar_lb;
                motion_matrix_ub = A_bar_ub + B_bar_ub;
                number_of_motion_cone_constraints = size(D, 1);
                A_constraint = zeros(number_of_motion_cone_constraints, optimization_problem.nv);
                A_constraint(:, optimization_problem.vars.x.i(:, step_index)) = E;
                [~, E_ub] = obj.GetBoundsMatrixTimesVector(E, current_x_lb, current_x_ub);
                [~, D_ub] = obj.GetBoundsMatrixTimesVector(D, current_u_lb, current_u_ub);
                constraint_matrix_ub = E_ub + D_ub;
            end
            M_motion_ub = current_x_ub - motion_matrix_lb - b_motion;
            M_motion_lb = current_x_lb - motion_matrix_ub - b_motion;
            b_motion1 = b_motion + M_motion_ub;
            b_motion2 = -b_motion - M_motion_lb;
            A_motion1 = A_motion;
            A_motion2 = -A_motion;
            A_motion1(:, optimization_problem.vars.z.i(state_index, ceil(step_index ./ obj.clustering_factor))) = M_motion_ub;
            A_motion2(:, optimization_problem.vars.z.i(state_index, ceil(step_index ./ obj.clustering_factor))) = -M_motion_lb;
            A_constraint(:, optimization_problem.vars.u.i(:, step_index)) = D;
            M_constraint_ub = constraint_matrix_ub - g;
            A_constraint(:, optimization_problem.vars.z.i(state_index, ceil(step_index ./ obj.clustering_factor))) = M_constraint_ub;
            b_constraint = g + M_constraint_ub;
            optimization_problem = optimization_problem.addLinearConstraints([A_motion1; A_motion2; A_constraint], [b_motion1; b_motion2; b_constraint], [], []);
            clear A_motion1 A_motion2 b_motion1 b_motion2 A_constraint b_constraint M_motion_ub M_motion_lb M_constraint_ub
        end
        %% sum(z) == 1
        A_z = zeros(1, optimization_problem.nv);
        A_z(1, optimization_problem.vars.z.i(:, ceil(step_index ./ obj.clustering_factor))) = 1 * ones(1, obj.number_of_states);
        b_z = 1;
        optimization_problem = optimization_problem.addLinearConstraints([], [], A_z, b_z);
    end
end
end
methods (Static)
function [a_lb, a_ub] = GetBoundsValueTimesX(a, x_lb, x_ub)
    if sign(a) >= 0
        a_lb = a * x_lb;
        a_ub = a * x_ub;
    else
        a_lb = a * x_ub;
        a_ub = a * x_lb;
    end
end

function [A_row_lb, A_row_ub] = GetBoundsMatrixRowTimesVector(A_row, x_lb, x_ub)
    assert(length(A_row) == length(x_lb), 'length A_row and length of x_lb mismatch');
    A_row_lb = 0;
    A_row_ub = 0;
    for i = 1:length(A_row)
        [A_row_lb_diff, A_row_ub_diff] = MPCSolvers.MIQPSolver.GetBoundsValueTimesX(A_row(i), x_lb(i), x_ub(i));
        A_row_lb = A_row_lb + A_row_lb_diff;
        A_row_ub = A_row_ub + A_row_ub_diff;
    end
end

function [A_lb, A_ub] = GetBoundsMatrixTimesVector(A, x_lb, x_ub)
    assert(size(A, 2) == length(x_lb), 'A number of rows and length of x_lb mismatch');
    assert(length(x_ub) == length(x_lb), 'length of x_ub and length of x_lb mismatch');
    A_lb = zeros(size(A, 1), 1);
    A_ub = zeros(size(A, 1), 1);
    for i = 1:size(A, 1)
        [A_lb(i), A_ub(i)] = MPCSolvers.MIQPSolver.GetBoundsMatrixRowTimesVector(A(i, :), x_lb, x_ub);
    end
end
end
end