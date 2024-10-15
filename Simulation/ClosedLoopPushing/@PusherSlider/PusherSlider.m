classdef PusherSlider < dynamicprops
    properties (Constant)
        %Pusher constants
        a = 0.068; %0.082; % length of the slider
        b = 0.082; %0.0625; % width of the slider 
        height = 0.041; %0.03;
        A = PusherSlider.a * PusherSlider.b; %area
        V = PusherSlider.A * PusherSlider.height; %volume
        nu = 0.32; %0.25 %mu_g: coefficient of friction slider-table
        nu_p = 0.19; %0.7; %0.1951; %mu_p: coefficient of friction slider-pusher
%         rho = 10000; % densità di massa slider
        m =  0.2875; %massa 0.46;PusherSlider.rho * PusherSlider.V;
 
        f_max = (PusherSlider.nu * PusherSlider.m * Helper.g);
        m_max = PusherSlider.m_max_funct(PusherSlider.nu, PusherSlider.m);
        c = PusherSlider.m_max / PusherSlider.f_max; 
        
        sample_delay = 0*5;%3;
        num_delayed = 2*PusherSlider.sample_delay;  %delayed samples
        
        %Perturbated constant
        m_pert = PusherSlider.m * 1.0; 
        nu_pert = PusherSlider.nu *1.0; 
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
        h_opt = 0.050; % sample time - PASSO DI CAMPIONAMENTO DISCRETIZZAZIONE MATRICI
        %h_step = 0.01; % sample time to discretize with Euler - INUTILE!
        steps = 10; %round(5e-3/(PusherSlider.h_opt*PusherSlider.u_star(1))); %20; % prediction horizon
        NumFam = 3; % number of families of modes
        num_int = 1; 
        num_vars = 4; % state [x y theta ry] ???
        num_xvars = 3; % [x y theta]
        num_dvars = 1; % [ry]
        num_inputs = 2; % u = [u1 u2]
        
        % %%%%%%%%%%% WORKING MATRICES WITH NU_P 0.19 %%%%%%%%%%%%%%%%
        %*PusherSlider.u_star(1)/5e-3,
        %Matrici buone
%         W_x = 0.01*diag([100 100 0.1 0]);  % State matrix weight
%         W_x_e = 200*diag([1000 1000 0.1 0]); %diag([100 100 0 0 0]);
%         W_u = diag([1e-3 1e-3]);
        Q_MPC = 0.01*diag([100 100 0.1 0]);%10* diag([0.5,500*PusherSlider.u_star(1),0.005,0.01]); % 10 * diag([1,10,10,0]);
        Q_MPC_final =  200*diag([1000 1000 0.1 0]);%200* diag([3,3,0.03,0.0]); % 200 *diag([.1,10,1,0]); buone
        R_MPC = diag([1e-3 1e-3]); %1 * diag([1,1]);
%         Q_MPC = 1*diag([100 100 0.001 0]);%10* diag([0.5,500*PusherSlider.u_star(1),0.005,0.01]); % 10 * diag([1,10,10,0]);
%         Q_MPC_final =  200*diag([1000 1000 1 0]);%200* diag([3,3,0.03,0.0]); % 200 *diag([.1,10,1,0]); buone
%         R_MPC = 10*diag([10 1]); %1 * diag([1,1]);
        
%         Q_MPC = 10* diag([1,18,.2,0]);
%         Q_MPC_final =  150* diag([1,18,.2,0.0]);
%         R_MPC = 1 * diag([1,2]);
        

        % Declare Equilibrium variables (nominal variables)
        u_star = [0.01;0];
        ry_star= [0];
        x_eq= [0;0;0];
        u_lower_bound = [-0.01; -0.05]; % constraint on velocity u where u = upert + ustar
        u_upper_bound = [0.03; 0.05];
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
        Ani; % Ajn in (19)
        t; 
        x_state;
        u_state;
        delta_u_state; % u perturbated
        delta_u_pert; % delta_u(k) - delta_u(k-1)
        u_prev_pert = [0 0];
        vipi_state;
        Figures;
        FiguresMPC;
        x_star;
        gammaTop_star;
        gammaBottom_star;
        K;
        A_linear; % Aj in (9)
        B_linear;% Bj in (9)
        C_top_linear; % Ct in (11)
        C_bottom_linear; % Cb in (11)
        A_bar;
        B_bar;
        Opt;
        numDynConstraints; %dynamics constraints
        numMCConstraints; %motion cone constraints 
        NumSim;
        FOM;
        MIQP;
        binInitial; %Bj0 in (17)
        AinInitial; %
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
        f_star; %(initial f in equilibrium state)
        CostVector;
        starIndex;
        ControllerType;
        flag = 0;
        t0 = 0;
        traj_x0 = zeros(3,1)
        lin_vel_star = PusherSlider.u_star(1);
        rot_vel_star = 2*deg2rad(2.5);
        solutionVec = [];
        solutionVecNoFilter = [];
        index = [];
        u_old = [];
        delay_u_pert = zeros(2,PusherSlider.sample_delay);
        deltau_st = [];
        deltau_su = [];
        deltau_sd = [];
        deltau = [];
        x_state_vec_sim = [];
        x_state_vec = [];
        x_star_vect = [];
        u_control_vec = [];
        t_vec = [];
        rpusher_vec =[];
        start_time_ros = 0;
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
%                 disp('Error: Could not find proper flag string');
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
%             obj.hybrid_modes = [ones(1, obj.steps); 2, ones(1, obj.steps - 1); 3, ones(1, obj.steps - 1)];
            num_sliding = 1;%ceil(3*obj.steps/20);%1;
            obj.hybrid_modes = [ones(1, obj.steps); 2*ones(1,num_sliding), ones(1, obj.steps - num_sliding); 3*ones(1,num_sliding), ones(1, obj.steps - num_sliding)];
            obj.fom_solver= MPCSolvers.FOMSolver(obj.hybrid_states_map, obj.Q_MPC, obj.Q_MPC_final, obj.R_MPC, obj.u_lower_bound, obj.u_upper_bound, obj.x_lower_bound, obj.x_upper_bound, obj.h_opt, obj.hybrid_modes, true);
            %obj.euler_integrator = EulerIntegration(obj.real_states_map, obj.h_step);
        end
    end
    methods (Static, Access = private)
        function n_f = m_max_funct(nu, m)     
            n_f_integrand = @(p1, p2) (nu * m * Helper.g / PusherSlider.A) * sqrt([p1; p2; 0]' * [p1; p2; 0]);
            n_f = Helper.DoubleGaussQuad(n_f_integrand, -PusherSlider.a / 2, PusherSlider.a / 2, -PusherSlider.b / 2, PusherSlider.b / 2);
        end     
    end
end