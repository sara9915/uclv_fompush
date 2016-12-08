%% Build Constraint Matrices (FOM)
% TODO: A lot of repeated code with ConstraintMatricesFOM
function obj = ConstraintMatricesMIQP(obj)
    M=100;
    %Define discrete linear dynamic matrices
    for lv1=1:3
        obj.A_bar{lv1} = eye(length(obj.A_linear{lv1})) + obj.h_opt*obj.A_linear{lv1};
        obj.B_bar{lv1} = obj.h_opt*obj.B_linear{lv1};
    end
    %Define optimization program
    MIQP = MixedIntegerConvexProgram(false);
    MIQP = MIQP.addVariable('u', 'C', [obj.num_inputs, obj.steps], -100*ones(obj.num_inputs,obj.steps), 100*ones(obj.num_inputs,obj.steps));
    MIQP = MIQP.addVariable('x', 'C', [obj.num_vars, obj.steps], -100*ones(obj.num_vars,obj.steps), 100*ones(obj.num_vars,obj.steps));
    MIQP = MIQP.addVariable('region', 'B', [3, obj.num_int], 0, 1);
    
    %Define default mode
    zStick = [1,0,0];
    counterStick = 1;
    %Loop through steps of opt. program
    for lv1=1:obj.steps;
        %% Define Cost Functions
        H = zeros(MIQP.nv, MIQP.nv);
        c = zeros(MIQP.nv, 1);
        % Assign cost block matrices
        H(MIQP.vars.x.i(1:length(obj.Q_MPC),lv1), MIQP.vars.x.i(1:length(obj.Q_MPC),lv1)) = zeros(length(obj.Q_MPC));
        H(MIQP.vars.x.i(1:length(obj.Q_MPC),lv1), MIQP.vars.x.i(1:length(obj.Q_MPC),lv1)) = obj.Q_MPC;
        H(MIQP.vars.u.i(1:length(obj.R_MPC),lv1), MIQP.vars.u.i(1:length(obj.R_MPC),lv1)) = obj.R_MPC;
        if lv1==1
        else
            H(MIQP.vars.u.i(1,lv1), MIQP.vars.u.i(1,lv1)) = obj.R_switch;
            H(MIQP.vars.u.i(1,lv1), MIQP.vars.u.i(1,lv1-1)) = -obj.R_switch;
            H(MIQP.vars.u.i(1,lv1-1), MIQP.vars.u.i(1,lv1)) = -obj.R_switch;
            H(MIQP.vars.u.i(1,lv1-1), MIQP.vars.u.i(1,lv1-1)) = obj.R_switch;

            H(MIQP.vars.u.i(2,lv1), MIQP.vars.u.i(2,lv1)) = obj.R_switch;
            H(MIQP.vars.u.i(2,lv1), MIQP.vars.u.i(2,lv1-1)) = -obj.R_switch;
            H(MIQP.vars.u.i(2,lv1-1), MIQP.vars.u.i(2,lv1)) = -obj.R_switch;
            H(MIQP.vars.u.i(2,lv1-1), MIQP.vars.u.i(2,lv1-1)) = obj.R_switch;
        end
        %Final Cost
        if lv1 == obj.steps
            H(MIQP.vars.x.i(1:length(obj.Q_MPC),lv1), MIQP.vars.x.i(1:length(obj.Q_MPC),lv1)) = obj.Q_MPC_final;
        end
        MIQP = MIQP.addCost(H, [], []); 
       
        %% Define mode dependant constraints
        if lv1<=obj.num_int
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Dynamic Constraints
            for counter = 1:3
                Ain1 = zeros(obj.num_vars, MIQP.nv);
                if lv1 ==1
                else
                    Ain1(:,MIQP.vars.x.i(1:obj.num_vars,lv1-1)) = -obj.A_bar{counter};
                end
                Ain1(:,MIQP.vars.x.i(1:obj.num_vars,lv1))   = eye(obj.num_vars);
                Ain1(:,MIQP.vars.u.i(1:obj.num_inputs,lv1)) =  -obj.B_bar{counter};
                
                bin1 = ones(obj.num_vars,1)*M*(1);
                bin2 = bin1;
                Ain2 = -Ain1;
                Ain1(:,MIQP.vars.region.i(counter,lv1)) = M;
                Ain2(:,MIQP.vars.region.i(counter,lv1)) = M;

                MIQP = MIQP.addLinearConstraints(Ain1, bin1, [], []);
                MIQP = MIQP.addLinearConstraints(Ain2, bin2, [], []);
                clear Ain1 Ain2 bin1 bin2
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Motion Cone Constraints
            %% Motion Cone Constraint 
            epsilon = 0.005;
            %Sticking MC (2 constraints)
            NconstMC = 1;
            D = [-obj.gammaTop_star 1];
            E = -obj.C_top_linear*obj.u_star(2);
            bin = M*(1)-obj.u_star(2) + obj.gammaTop_star*obj.u_star(1);
            Ain = zeros(NconstMC, MIQP.nv);
            Ain(:,MIQP.vars.x.i(1:obj.num_vars,lv1)) = E;
            Ain(:,MIQP.vars.u.i(1:2,lv1)) = D;
            Ain(:,MIQP.vars.region.i(1,lv1)) = M;
            MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);   
            clear D E Ain bin
            D = [obj.gammaBottom_star -1];
            E = obj.C_bottom_linear*obj.u_star(2);
            bin = M*(1)+obj.u_star(2) - obj.gammaBottom_star*obj.u_star(1);
            Ain = zeros(NconstMC, MIQP.nv);
            Ain(:,MIQP.vars.x.i(1:obj.num_vars,lv1)) = E;
            Ain(:,MIQP.vars.u.i(1:2,lv1)) = D;
            Ain(:,MIQP.vars.region.i(1,lv1)) = M;
            MIQP = MIQP.addLinearConstraints(Ain, bin, [], []); 
            clear D E Ain bin
            %Sliding up MC (1 constraint)
            D = [obj.gammaTop_star -1];
            E = obj.C_top_linear*obj.u_star(2);
            bin = M*(1)+obj.u_star(2) - obj.gammaTop_star*obj.u_star(1)-epsilon;
            Ain = zeros(NconstMC, MIQP.nv);
            Ain(:,MIQP.vars.x.i(1:obj.num_vars,lv1)) = E;
            Ain(:,MIQP.vars.u.i(1:2,lv1)) = D;
            Ain(:,MIQP.vars.region.i(2,lv1)) = M;
            MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);   
            clear D E Ain bin
            %Sliding down MC (1 constraint)
            D = [-obj.gammaBottom_star 1];
            E = -obj.C_bottom_linear*obj.u_star(2);
            bin = M*(1)-obj.u_star(2) + obj.gammaBottom_star*obj.u_star(1)-epsilon;
            Ain = zeros(NconstMC, MIQP.nv);
            Ain(:,MIQP.vars.x.i(1:obj.num_vars,lv1)) = E;
            Ain(:,MIQP.vars.u.i(1:2,lv1)) = D;
            Ain(:,MIQP.vars.region.i(3,lv1)) = M;
            MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);   
            clear D E Ain bin
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % sum(region) == 1
            Aeq_comp = zeros(1, MIQP.nv);
            Aeq_comp(1, MIQP.vars.region.i(:,lv1)) = 1;
            beq_comp = 1;
            MIQP = MIQP.addLinearConstraints([], [], Aeq_comp, beq_comp);

        else %FOM Formulation (Assume sticking for rest of simulation)
            %% Dynamic Constraints
            Ain1 = zeros(obj.num_vars, MIQP.nv);
            if lv1 ==1
            else
                Ain1(:,MIQP.vars.x.i(1:obj.num_vars,lv1-1))= -obj.A_bar{counterStick};
            end
            Ain1(:,MIQP.vars.x.i(1:obj.num_vars,lv1))  = eye(obj.num_vars);
            Ain1(:,MIQP.vars.u.i(1:obj.num_inputs,lv1)) =  -obj.B_bar{counterStick};
            Ain1(:,MIQP.vars.x.i(1:obj.num_vars,lv1))  = eye(obj.num_vars);
            Ain1(:,MIQP.vars.u.i(1:obj.num_inputs,lv1)) =  -obj.B_bar{counterStick};
            bin1 = ones(obj.num_vars,1)*M*(1-zStick(counterStick));
            Ain2 = -Ain1;
            bin2 = bin1;

            MIQP = MIQP.addLinearConstraints(Ain1, bin1, [], []);
            MIQP = MIQP.addLinearConstraints(Ain2, bin2, [], []);
            clear Ain1 Ain2 bin1 bin2

            %% Motion Cone Constraint 
            epsilon = 0.005;
            if counterStick==1
                %Sticking MC (2 constraints)
                NconstMC = 1;
                D = [-obj.gammaTop_star 1];
                E = -obj.C_top_linear*obj.u_star(2);
                bin = M*(1-zStick(1))-obj.u_star(2) + obj.gammaTop_star*obj.u_star(1);
                Ain = zeros(NconstMC, MIQP.nv);
                Ain(:,MIQP.vars.x.i(1:obj.num_vars,lv1)) = E;
                Ain(:,MIQP.vars.u.i(1:2,lv1)) = D;
                MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);   
                clear D E Ain bin
                D = [obj.gammaBottom_star -1];
                E = obj.C_bottom_linear*obj.u_star(2);
                bin = M*(1-zStick(1))+obj.u_star(2) - obj.gammaBottom_star*obj.u_star(1);
                Ain = zeros(NconstMC, MIQP.nv);
                Ain(:,MIQP.vars.x.i(1:obj.num_vars,lv1)) = E;
                Ain(:,MIQP.vars.u.i(1:2,lv1)) = D;
                MIQP = MIQP.addLinearConstraints(Ain, bin, [], []); 
                clear D E Ain bin
            elseif counterStick==2
                %Sliding up MC (1 constraint)
                D = [obj.gammaTop_star -1];
                E = obj.C_top_linear*obj.u_star(2);
                bin = M*(1-zStick(2))+obj.u_star(2) - obj.gammaTop_star*obj.u_star(1)-epsilon;
                Ain = zeros(NconstMC, MIQP.nv);
                Ain(:,MIQP.vars.x.i(1:obj.num_vars,lv1)) = E;
                Ain(:,MIQP.vars.u.i(1:2,lv1)) = D;
                MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);   
                clear D E Ain bin
            else
                %Sliding down MC (1 constraint)
                D = [-obj.gammaBottom_star 1];
                E = -obj.C_bottom_linear*obj.u_star(2);
                bin = M*(1-zStick(3))-obj.u_star(2) + obj.gammaBottom_star*obj.u_star(1)-epsilon;
                Ain = zeros(NconstMC, MIQP.nv);
                Ain(:,MIQP.vars.x.i(1:obj.num_vars,lv1)) = E;
                Ain(:,MIQP.vars.u.i(1:2,lv1)) = D;
                MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);   
                clear D E Ain bin
            end
        end
                 %% Define mode-independant constraints
        %Vn min
        Ain = zeros(1, MIQP.nv);
        Ain(:,MIQP.vars.u.i(1,lv1)) = -1;
        bin = obj.u_star(1)-0.03;
        MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);  
        clear Ain bin 
        %Vn max
        Ain = zeros(1, MIQP.nv);
        Ain(:,MIQP.vars.u.i(1,lv1)) = 1;
        bin = -obj.u_star(1)+0.1;
        MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);  
        clear Ain bin
        %% Vt min
        Ain = zeros(1, MIQP.nv);
        Ain(:,MIQP.vars.u.i(2,lv1)) = -1;
        bin = obj.u_star(2)+0.1;
        MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);  
        clear Ain bin
        %% Vt max
        Ain = zeros(1, MIQP.nv);
        Ain(:,MIQP.vars.u.i(2,lv1)) = 1;
        bin = -obj.u_star(2)+0.1;
        MIQP = MIQP.addLinearConstraints(Ain, bin, [], []);  
        clear Ain bin
        %Define struct to store initial value for bin (before IC added
        %at each time step)
        obj.binInitial = MIQP.b;
    end
    %Save data 
    obj.Opt.MIQP = MIQP;
end