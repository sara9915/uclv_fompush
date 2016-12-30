classdef PusherSlider < dynamicprops
    properties (Constant)
        %Pusher constants
        a = 0.09;
        b = 0.09;
        nu = 0.35;
        nu_p = 0.3;
        rho = 10000;
        height = 0.013;
        %LQR variables
        Q_LQR = diag([1,1,.1]);
        R_LQR = .1 * eye(2);
        %Euler Integration
        num_states = 5;
        %Optimization Program
        h_opt = 0.03;
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
    end
    properties(Access = private)
        hybrid_states_map;
        hybrid_modes = HybridMode;
    end
    properties
       Ani;
       c;
       m;
       t;
       V; 
       A; 
       m_max;
       f_max;
       c_pert;
       m_pert;
       m_max_pert;
       f_max_pert;
       nu_pert;
       nu_p_pert;
       x_state;
       u_state;
       delta_u_state;
       vipi_state;
       Figures;
       FiguresMPC;
       x_eq;
       x_star;
       ry_star;
       gammaTop_star;
       gammaBottom_star;
       u_star;
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
            %Set constant equations
            obj.A = obj.a*obj.b;
            obj.V = obj.A*obj.height;
            obj.m = obj.rho*obj.V;
            %Compute f_max and m_max
            obj.f_max = (obj.nu*obj.m*Helper.g);
            obj.m_max = obj.m_max_funct(obj.nu, obj.m);
            obj.c = obj.m_max/obj.f_max; 
            %Compute perturbed states
            obj.m_pert = obj.m*1.0; 
            obj.nu_pert = obj.nu*1.0; 
            obj.nu_p_pert = obj.nu_p*1.0; 
            obj.f_max_pert = (obj.nu_pert*obj.m_pert*Helper.g);
            obj.m_max_pert = obj.m_max_funct(obj.nu_pert, obj.m_pert);
            obj.c_pert = obj.m_max_pert/obj.f_max_pert; 
            %Set proper controller type
            obj.ControllerType = flag;
            if strcmp(flag,'Trajectory')
                obj.starIndex = 1;
            elseif strcmp(flag,'Target')
                obj.starIndex = 2;
            else
                disp('Error: Could not find proper flag string');
            end
            c2 = obj.c * obj.c;
            obj.hybrid_states_map = horzcat(StickingState(obj.a, obj.nu_p, c2), ...
                                        SlidingUpState(obj.a, obj.nu_p, c2), ...
                                        SlidingDownState(obj.a, obj.nu_p, c2));
            for i=1:length(obj.hybrid_states_map)
                obj.hybrid_states_map(i).SymbolicLinearize();
            end
            % TODO: Add capability to chose more Modes and even to make it
            % automatically
            obj.hybrid_modes(1,3) = HybridMode;
            obj.hybrid_modes(1,1) = HybridMode(ones(1, obj.steps), obj.num_vars, obj.num_inputs);
            obj.hybrid_modes(1,2) = HybridMode([2, ones(1, obj.steps - 1)], obj.num_vars, obj.num_inputs);
            obj.hybrid_modes(1,3) = HybridMode([3, ones(1, obj.steps - 1)], obj.num_vars, obj.num_inputs);
        end
    end
end