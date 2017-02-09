%% Clear
clear variables; % Clear all is bad for performance
close all;
clc;
% Setup
run('Setup.m');

%% Simulation Parameters
t0 = 0;
tf = 5;
tic;
%% Build PusherSlider
p = PusherSlider('Trajectory');%'Target'
p = p.symbolicLinearize();
p = p.symbolicNonLinearize();
p = p.ConstraintMatrices('FOM');
x_star_test = [p.x_eq(1); p.x_eq(2); p.x_eq(3); p.ry_star];
u_star_test = [p.u_star(1); p.u_star(2)];
for i = 1:3
    disp(i);
    [A, B, D, E, g] = p.hybrid_states_map(i).GetLinearMatrices(x_star_test, u_star_test);
    real_A = p.A_linear{i};
    diff_A = abs(A - real_A);
    assert(all(diff_A(:) < 1e-10), 'A matrices mismatch');
    real_B = p.B_linear{i}; 
    diff_B = abs(B - real_B);
    assert(all(diff_B(:) < 1e-10), 'B matrices mismatch');
    for j = 1:100
        x = rand(4,1);
        u = rand(2,1);
        x0 = rand(4,1);
        [B_nonlinear, F, D, g] = p.hybrid_states_map(i).GetInitialStateMatrices(x0, x, u);
        f = p.hybrid_states_map(i).GetMotionFunction(x, u);
        real_B_nonlinear = p.B_Nonlinear{i}(x0(4), x0(3));
        diff_B_nonlinear = abs(B_nonlinear - real_B_nonlinear);
        assert(all(diff_B_nonlinear(:) < 1e-10), 'B_nonlinear functions mismatch');
        real_f = p.f_star{i}(x(4), x(3), u(1), u(2));
        diff_f = abs(f - real_f);
        assert(all(diff_f(:) < 1e-10), 'motion function mismatch');
        is_inside = p.hybrid_states_map(i).CheckConstraints(x, u);
    end
end
u_star = @(t)([PusherSlider.u_star(1) + 0 .*t; PusherSlider.u_star(2) + 0 .*t]); % To ensure it can be substituted by a vector
x_star = @(t)([PusherSlider.u_star(1) .* t; 0.*t; 0.*t; 0.*t]);
x0 = [0, .05, 30 * pi / 180, 0];
tic;
p.EulerIntegration(t0, tf, p.h_step, x0(1, 4), x0(1,1:3).', 1);
toc;
old_x_state = p.x_state{2}(:,1:2).';
tic;
[new_x_state, new_u_state, new_x_bar, new_u_bar, new_t, new_modes, new_costs] = p.euler_integrator.IntegrateMPC(t0, tf, x0, x_star, u_star, p.fom_solver);
toc;
diff_state = old_x_state - new_x_state(1:2,:);
err_state = sqrt(diff_state(1,:).^2 + diff_state(2,:).^2);
assert(max(err_state) < 1e-3, 'New solution differs from previous one');