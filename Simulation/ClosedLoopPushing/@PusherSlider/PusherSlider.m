classdef PusherSlider < dynamicprops
    properties (Constant)
        %Pusher constants
        a = 0.09;
        b = 0.09;
        A = PusherSlider.a * PusherSlider.b;
        V = PusherSlider.A * PusherSlider.height;
        nu = 0.35;
        nu_p = 0.3;
        rho = 10000;
        m = PusherSlider.rho * PusherSlider.V;
        height = 0.013;
        f_max = (PusherSlider.nu * PusherSlider.m * Helper.g);
        m_max = PusherSlider.m_max_funct(PusherSlider.nu, PusherSlider.m);
        c = PusherSlider.m_max / PusherSlider.f_max; 
        m_pert = PusherSlider.m * 1.0; 
        nu_pert = PusherSlider.nu * 1.0; 
        nu_p_pert = PusherSlider.nu_p * 1.0; 
        f_max_pert = (PusherSlider.nu_pert * PusherSlider.m_pert * Helper.g);
        m_max_pert = PusherSlider.m_max_funct(PusherSlider.nu_pert, PusherSlider.m_pert);
        c_pert = PusherSlider.m_max_pert / PusherSlider.f_max_pert;
        %LQR variables
        Q_LQR = diag([1,1,.1]);
        R_LQR = .1 * eye(2);
        %Euler Integration
        num_states = 5;
        %Optimization Program
        h_opt = 0.03;
        h_step = 0.01;  
%         steps = 2; 
        steps = 35;
        NumFam = 3;
        num_int = 1;
        num_vars = 4;
        num_xvars = 3;
        num_dvars = 1;
        num_inputs = 2;
        Q_MPC = 10 * diag([1,3,.1,0]); % 10 * diag([1,10,10,0]);
        Q_MPC_final = 200 * diag([1,3,.1,0]); % 200 * diag([.1,10,1,0]);
        R_MPC = .5 * diag([1,1]);
        R_switch = 0;
        % Declare Equilibrium variables
        u_star = [0.05;0];
        ry_star= [0];
        x_eq= [0;0;0];
        u_lower_bound = [-0.01; 0.1];
        u_upper_bound = [0.1; 0.1];
        x_lower_bound = [100; 100; 100; 100];
        x_upper_bound = [100; 100; 100; 100];
    end
    properties
%     properties(Access = private) % TODO: Undo
        hybrid_states_map;
        hybrid_modes;
        real_states_map;
        fom_solver;
        euler_integrator;
    end
    properties
        Ani;
        t; 
        x_state;
        u_state;
        delta_u_state;
        vipi_state;
        Figures;
        FiguresMPC;
        x_star;
        gammaTop_star;
        gammaBottom_star;
        K;
        A_linear;
        B_linear;
        C_top_linear;
        C_bottom_linear;
        A_bar;
        B_bar;
        Opt;
        numDynConstraints;
        numMCConstraints;
        NumSim;
        FOM;
        MIQP;
        binInitial;
        AinInitial;
        beqInitial;
        AeqInitial;
        Cost;
        StateCost;
        x0index;
        u0index;
        delta_u_prev = [0;0];
        delta_u_delay = [0;0];
        modes = [];
        B_Nonlinear;
        f_star;
        CostVector;
        starIndex;
        ControllerType;
        flag = 0;
    end
    methods
        %% Constructor
        function obj = PusherSlider(flag)     
            %Set proper controller type
            obj.ControllerType = flag;
            if strcmp(flag,'Trajectory')
                obj.starIndex = 1;
            elseif strcmp(flag,'Target')
                obj.starIndex = 2;
            else
                disp('Error: Could not find proper flag string');
            end
            c2 = PusherSlider.c^2;
            obj.hybrid_states_map = horzcat(PusherSliderStates.QSSticking(PusherSlider.a, PusherSlider.nu_p, c2), ...
                                            PusherSliderStates.QSSlidingUp(PusherSlider.a, PusherSlider.nu_p, c2), ...
                                            PusherSliderStates.QSSlidingDown(PusherSlider.a, PusherSlider.nu_p, c2));
            c2_pert = PusherSlider.c_pert^2;
            obj.real_states_map = horzcat(PusherSliderStates.QSSticking(PusherSlider.a, PusherSlider.nu_p_pert, c2_pert), ...
                                          PusherSliderStates.QSSlidingUp(PusherSlider.a, PusherSlider.nu_p_pert, c2_pert), ...
                                          PusherSliderStates.QSSlidingDown(PusherSlider.a, PusherSlider.nu_p_pert, c2_pert));
            % TODO: Add capability to chose more Modes and even to make it
            % automatically
            obj.hybrid_modes = [ones(1, obj.steps); 2, ones(1, obj.steps - 1); 3, ones(1, obj.steps - 1)];
            obj.fom_solver= MPCSolvers.FOMSolver(obj.hybrid_states_map, obj.Q_MPC, obj.Q_MPC_final, obj.R_MPC, obj.u_lower_bound, obj.u_upper_bound, obj.x_lower_bound, obj.x_upper_bound, obj.h_opt, obj.hybrid_modes);
            obj.euler_integrator = EulerIntegration(obj.real_states_map, obj.h_step);
        end
    end
    methods (Static, Access = private)
        function n_f = m_max_funct(nu, m)     
            n_f_integrand = @(p1, p2) (nu * m * Helper.g / PusherSlider.A) * sqrt([p1; p2; 0]' * [p1; p2; 0]);
            n_f = Helper.DoubleGaussQuad(n_f_integrand, -PusherSlider.a / 2, PusherSlider.a / 2, -PusherSlider.b / 2, PusherSlider.b / 2);
        end     
    end
end