%% Build Constraint Matrices (FOM)
function obj = ConstraintMatricesFOM(obj)
    %json parameters for saving matrices
    load_json = true;
    filename_json = 'Matrices_1.json';
    if(load_json)
        A_json = [];
        B_json = [];
        H_json_R = [];
        H_json_Q = [];
        H_json_Qf = [];
    end
    
    for Family = 1:obj.NumFam
        M=100;
        %Define discrete linear dynamic matrices
        for lv1=1:3 % TODO: Ask Frank about lvl and the next lines
            obj.A_bar{lv1} = eye(length(obj.A_linear{lv1})) + obj.h_opt*obj.A_linear{lv1};
            obj.B_bar{lv1} = obj.h_opt*obj.B_linear{lv1};
        end
        %Define optimization program
        FOM{Family} = MixedIntegerConvexProgram(false);
%         u_lower_bound = [(-obj.u_star(1) + obj.u_lower_bound(1)) * ones(1, obj.steps); (-obj.u_star(2) - obj.u_lower_bound(2)) * ones(1, obj.steps)]; % TODO: Change for complex trajectories and include as member of something
%         u_upper_bound = [(-obj.u_star(1) + obj.u_upper_bound(1)) * ones(1, obj.steps); (-obj.u_star(2) + obj.u_upper_bound(2)) * ones(1, obj.steps)]; % TODO: Change for complex trajectories and include as member of something
        u_lower_bound = [(-obj.u_star(1) + 0.01) * ones(1, obj.steps); (-obj.u_star(2) - 0.1) * ones(1, obj.steps)]; % TODO: Change for complex trajectories and include as member of something
        u_upper_bound = [(-obj.u_star(1) + 0.1) * ones(1, obj.steps); (-obj.u_star(2) + 0.1) * ones(1, obj.steps)]; % TODO: Change for complex trajectories and include as member of something        
        x_lower_bound = [-100 * ones(3, obj.steps); -.75 * obj.a / 2 * ones(1, obj.steps)];
        x_upper_bound = [100 * ones(3, obj.steps); .75 * obj.a / 2 * ones(1, obj.steps)];
        FOM{Family} = FOM{Family}.addVariable('u', 'C', [obj.num_inputs, obj.steps], u_lower_bound, u_upper_bound);
        FOM{Family} = FOM{Family}.addVariable('x', 'C', [obj.num_vars, obj.steps], x_lower_bound, x_upper_bound);
        
        %Loop through steps of opt. program
        % TODO: Change name so it's not misleading anymore
        for lv1=1:obj.steps;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % TODO: Probably better to create a Family object
            if lv1 <2
                if Family==1 
                    z = [1,0,0];
                elseif Family==2
                    z = [0,1,0];
                else
                    z = [0,0,1];
                end
            else
                z = [1,0,0];
            end
            %Set counter for Linear matrices indexes
            % TODO: Better encapsulate this in a FOM object
            if z(1) == 1
                counter = 1;
            elseif z(2) == 1
                counter = 2;
            else
                counter = 3;
            end
            %% Cost - DEFINE QUADRATIC COST FUNCTION 
            H = zeros(FOM{Family}.nv, FOM{Family}.nv);
            c = zeros(FOM{Family}.nv, 1);
            % Assign cost block matrices
            if lv1 < obj.steps
                H(FOM{Family}.vars.x.i(1:length(obj.Q_MPC),lv1), FOM{Family}.vars.x.i(1:length(obj.Q_MPC),lv1)) = zeros(length(obj.Q_MPC));
                H(FOM{Family}.vars.x.i(1:length(obj.Q_MPC),lv1), FOM{Family}.vars.x.i(1:length(obj.Q_MPC),lv1)) = obj.Q_MPC;
            end
            H(FOM{Family}.vars.u.i(1:length(obj.R_MPC),lv1), FOM{Family}.vars.u.i(1:length(obj.R_MPC),lv1)) = obj.R_MPC;
            %Final Cost
            if lv1 == obj.steps
                H(FOM{Family}.vars.x.i(1:length(obj.Q_MPC_final),lv1), FOM{Family}.vars.x.i(1:length(obj.Q_MPC_final),lv1)) = obj.Q_MPC_final;
            end
            FOM{Family} = FOM{Family}.addCost(H, [], []); 
            
            if(load_json && Family == 1)
                if lv1 == obj.steps
                    H_json_Qf = [H_json_Qf; H(FOM{Family}.vars.x.i(1:length(obj.Q_MPC_final),lv1),:)];
                    H_json_R = [H_json_R; H(FOM{Family}.vars.u.i(1:length(obj.R_MPC),lv1),:)];
                else
                    H_json_R = [H_json_R; H(FOM{Family}.vars.u.i(1:length(obj.R_MPC),lv1),:)];
                    H_json_Q = [H_json_Q; H(FOM{Family}.vars.x.i(1:length(obj.Q_MPC),lv1),:)];
                end
            end
            
            %% Dynamic Constraints - EQUALITY CONSTRAINTS FOR DYNAMICS
            Ain = zeros(obj.num_vars, FOM{Family}.nv);
            if lv1 ==1
            else
                Ain(:,FOM{Family}.vars.x.i(1:obj.num_vars,lv1-1))= -obj.A_bar{counter};
                Ain(:,FOM{Family}.vars.x.i(1:obj.num_vars,lv1))  = eye(obj.num_vars);
                Ain(:,FOM{Family}.vars.u.i(1:obj.num_inputs,lv1))=  -obj.B_bar{counter};
                bin = ones(obj.num_vars,1)*M*(1-z(counter));
                FOM{Family} = FOM{Family}.addLinearConstraints([], [], Ain, bin);
                
                if(load_json)
                     A_json = [A_json;Ain;-Ain]; 
                     B_json = [B_json;bin;bin];
                end
               
                clear Ain bin
                %% Motion Cone Constraint - INEQUALITY CONSTRAINTS
                epsilon = 0.005;
                NconstMC = 1; % TODO: Seems off, it's used in other ifs without initialization, so I moved out of the if/else
                Ain = zeros(NconstMC, FOM{Family}.nv);
                if counter==1
                    %Sticking MC (2 constraints)
                    D = [-obj.gammaTop_star 1];
                    E = -obj.C_top_linear*obj.u_star(1);
                    bin = M*(1-z(1))-obj.u_star(2) + obj.gammaTop_star*obj.u_star(1);
                    Ain(:,FOM{Family}.vars.x.i(1:obj.num_vars,lv1)) = E;
                    Ain(:,FOM{Family}.vars.u.i(1:2,lv1)) = D;
                    FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []);
                    
                    if(load_json)
                         A_json = [A_json;Ain]; 
                         B_json = [B_json;bin];
                    end
                    
                    clear D E Ain bin
                    D = [obj.gammaBottom_star -1];
                    E = obj.C_bottom_linear*obj.u_star(1);
                    bin = M*(1-z(1))+obj.u_star(2) - obj.gammaBottom_star*obj.u_star(1);
                    Ain = zeros(NconstMC, FOM{Family}.nv);
                    % TODO: Why not clear here?
                elseif counter==2
                    %Sliding up MC (1 constraint)
                    D = [obj.gammaTop_star -1];
                    E = obj.C_top_linear*obj.u_star(1);
                    bin = M*(1-z(2))+obj.u_star(2) - obj.gammaTop_star*obj.u_star(1)-epsilon;
                else
                    %Sliding down MC (1 constraint)
                    D = [-obj.gammaBottom_star 1];
                    E = -obj.C_bottom_linear*obj.u_star(1);
                    bin = M*(1-z(3))-obj.u_star(2) + obj.gammaBottom_star*obj.u_star(1)-epsilon;
                end
                Ain(:,FOM{Family}.vars.x.i(1:obj.num_vars,lv1)) = E;
                Ain(:,FOM{Family}.vars.u.i(1:2,lv1)) = D;
                FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []);
                
                if(load_json)
                     A_json = [A_json;Ain]; 
                     B_json = [B_json;bin];
                end
                clear D E Ain bin
            end
%             %% Delta un max
%             if lv1==1
%                 Ain = zeros(1, FOM{Family}.nv);
%                 Ain(:,FOM{Family}.vars.u.i(1,lv1)) = 1;
%                 bin = 0.03;
%                 %Find associated indexes
%                 S1 = size(FOM{Family}.A);
%                 obj.u0index{Family,1} = [S1(1)+1 S1(1)+1];
%                 %Add constraints
%                 FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []); 
%                 clear Ain bin
%             else
%                 Ain = zeros(1, FOM{Family}.nv);
%                 Ain(:,FOM{Family}.vars.u.i(1,lv1)) = 1;
%                 Ain(:,FOM{Family}.vars.u.i(1,lv1-1)) = -1;
%                 bin = 0.03;
%                 FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []);  
%                 clear Ain bin
%             end
%             %% Delta ut max
%             if lv1==1
%                 Ain = zeros(1, FOM{Family}.nv);
%                 Ain(:,FOM{Family}.vars.u.i(2,lv1)) = 1;
%                 bin = 0.03;
%                 %Find associated indexes
%                 S1 = size(FOM{Family}.A);
%                 obj.u0index{Family,2} = [S1(1)+1 S1(1)+1];
%                 %Add constraints
%                 FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []); 
%                 clear Ain bin
%             else
%                 Ain = zeros(1, FOM{Family}.nv);
%                 Ain(:,FOM{Family}.vars.u.i(2,lv1)) = 1;
%                 Ain(:,FOM{Family}.vars.u.i(2,lv1-1)) = -1;
%                 bin = 0.03;
%                 FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []);  
%                 clear Ain bin
%             end
%             %% Delta un min
%             if lv1==1
%                 Ain = zeros(1, FOM{Family}.nv);
%                 Ain(:,FOM{Family}.vars.u.i(1,lv1)) = -1;
%                 bin = 0.03;
%                 %Find associated indexes
%                 S1 = size(FOM{Family}.A);
%                 obj.u0index{Family,3} = [S1(1)+1 S1(1)+1];
%                 %Add constraints
%                 FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []); 
%                 clear Ain bin
%             else
%                 Ain = zeros(1, FOM{Family}.nv);
%                 Ain(:,FOM{Family}.vars.u.i(1,lv1)) = -1;
%                 Ain(:,FOM{Family}.vars.u.i(1,lv1-1)) = 1;
%                 bin = 0.03;
%                 FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []);  
%                 clear Ain bin
%             end
%             %% Delta ut min
%             if lv1==1
%                 Ain = zeros(1, FOM{Family}.nv);
%                 Ain(:,FOM{Family}.vars.u.i(2,lv1)) = -1;
%                 bin = 0.03;
%                 %Find associated indexes
%                 S1 = size(FOM{Family}.A);
%                 obj.u0index{Family,4} = [S1(1)+1 S1(1)+1];
%                 %Add constraints
%                 FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []); 
%                 clear Ain bin
%             else
%                 Ain = zeros(1, FOM{Family}.nv);
%                 Ain(:,FOM{Family}.vars.u.i(2,lv1)) = -1;
%                 Ain(:,FOM{Family}.vars.u.i(2,lv1-1)) = 1;
%                 bin = 0.03;
%                 FOM{Family} = FOM{Family}.addLinearConstraints(Ain, bin, [], []);  
%                 clear Ain bin
%             end
%         end
        %% 
        %Define struct to store initial value for bin (before IC added
        %at each time step)
        obj.AinInitial{Family} = FOM{Family}.A;
        obj.binInitial{Family} = FOM{Family}.b;
        obj.AeqInitial{Family} = FOM{Family}.Aeq;
        obj.beqInitial{Family} = FOM{Family}.beq;
        

        
        %Save data 
        obj.Opt.FOM{Family} = FOM{Family};
        end
    
                %save json
        if(load_json)
            json_file.Matrices.Q = [H_json_R; H_json_Q; H_json_Qf];
            if(Family==1)
             json_file.Matrices.Ain1 = A_json;       
             json_file.Matrices.bin1 = B_json';
            else
                if(Family==2)
                  json_file.Matrices.Ain2 = A_json;       
                  json_file.Matrices.bin2 = B_json';   
                else
                    json_file.Matrices.Ain3 = A_json;       
                    json_file.Matrices.bin3 = B_json';
                    
                    encoded = jsonencode(json_file);
                    encoded = strrep(encoded, '{', sprintf('{\n\t'));
                    encoded = strrep(encoded, '}', sprintf('\n}'));
                    encoded = strrep(encoded, ':', sprintf(': '));
                    encoded = strrep(encoded, '[[', sprintf('[\n\t\t['));
                    encoded = strrep(encoded, ']],', sprintf(']\n\t],\n\t'));
                    encoded = strrep(encoded, '],"', sprintf(']\n\t],\n\t'));
                    encoded = strrep(encoded, '],[', sprintf('],\n\t\t['));
                    fid = fopen(filename_json,'w');
                    fprintf(fid,'%s',encoded);
                    fclose(fid);
                end 
            end          
            A_json = [];
            B_json = [];
        end
        
end