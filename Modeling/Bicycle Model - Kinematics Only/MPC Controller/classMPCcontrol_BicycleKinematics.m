% Lab 9 - MPC - Jamie Tsang Chow Chang - c3356366

classdef classMPCcontrol_BicycleKinematics < handle
    properties
        controlMode     % Indicator of active control mode
        u               % Previous control value
        T               % Sampling period of controller
        N_horizon       % MPC horizon length
        A               % Linearised A matrix for controller
        B               % Linearised B matrix for controller
        C_r             % Linearised C matrix for regulation output
        D_r             % Linearised D matrix for regulation output
        A_int           % A matrix augmented with integrator
        B_int           % B matrix augmented with integrator
        Q               % Q matrix for LQR cost
        R               % R matrix for LQR cost
        N               % N matrix for LQR cost
        N_x             % Feed-forward gain for states
        N_u             % Feed-forward gain for inputs
        y_ref           % Output regulation reference
        A_z             % State coefficient for integrator transition dynamics
        B_z             % Input coefficient for integrator transition dynamics
        z               % Integrator state

        A_MPC           % Concatenated A matrix over control horizon (Acal)
        B_MPC           % Concatenated B matrix over control horizon (Bcal)
        Q_MPC           % Concatenated Q matrix over control horizon (Qcal)
        R_MPC           % Concatenated R matrix over control horizon (Rcal)
        N_MPC           % Concatenated N matrix over control horizon (Ncal)
        H_MPC           % Hessian of quadratic control cost (Hcal)
        fBar_MPC        % Offset coefficient for quadratic control cost
        
        u_max           % Control positive limit
        u_min           % Control negative limit
        delta_u_max     % Control positive slew rate limit
        delta_u_min     % Control negative slew rate limit

        CONST_INEQUAL_A     % Inequality constraint A matrix
        CONST_INEQUAL_B     % Inequality constraint B matrix
        CONST_BOUND_UPPER   % Control upper bound vector
        CONST_BOUND_LOWER   % Control lower bound vector
    end



methods
       
    % MPC regulation initialisation
    function controlInit_regulation(obj,A,B,Q,R,N,T,N_horizon,u_max,u_min,delta_u_max,delta_u_min)
        % Store properties
        obj.A = A;
        obj.B = B;
        obj.Q = Q;
        obj.R = R;
        obj.T = T;
        obj.N_horizon = N_horizon;
        obj.u_max = u_max;
        obj.u_min = u_min;
        obj.delta_u_max = delta_u_max;
        obj.delta_u_min = delta_u_min;

        
        % Compute descrete time costs using the provided 'dlqr_cost'
        % function.
        [Ad,Bd,Qd,Rd,Nd] = dlqr_cost(A,B,Q,R,N,T);

        % Compute terminal cost Qf using 'dare' (or otherwise)
        [Qf,~,~] = dare(Ad,Bd,Qd,Rd,Nd,[]);

        % Construct concatinated matrices
        size_x = length(A);
        [~, size_u] = size(B);

        % Pre-allocate space for results
        obj.A_MPC = zeros(size_x*(N_horizon+1), size_x);
        obj.B_MPC = zeros(size_x*(N_horizon+1), N_horizon*size_u);
        obj.Q_MPC = zeros(size_x*(N_horizon+1), size_x*(N_horizon+1));
        obj.R_MPC = zeros(N_horizon*size_u, N_horizon*size_u);
        obj.N_MPC = zeros(size_x*(N_horizon+1), N_horizon*size_u);


        %% Construct matrices by looping through horizon
        obj.A_MPC(1:size_x,:) = eye(size_x);
        obj.Q_MPC(size_x*N_horizon+1:end, size_x*N_horizon+1:end) = Qf;

        for i=1:N_horizon
        obj.A_MPC(i*size_x+1:(i+1)*size_x,:) = Ad^i;
        obj.Q_MPC((i-1)*size_x+1:i*size_x, (i-1)*size_x+1:i*size_x) = Qd;
        obj.R_MPC(size_u*(i-1)+1:size_u*i, size_u*(i-1)+1:size_u*i) = Rd;
        obj.N_MPC((i-1)*size_x+1:i*size_x, size_u*(i-1)+1:size_u*i) = Nd;
        for j=1:i
        obj.B_MPC(i*size_x+1:(i+1)*size_x,size_u*(j-1)+1:size_u*j) = Ad^(i-j)*Bd;
        end
        end

        % Construct quadratic cost matrices
        % Hessian
        obj.H_MPC = 2*(obj.B_MPC.'*obj.Q_MPC*obj.B_MPC + obj.B_MPC.'*obj.N_MPC + obj.N_MPC.'*obj.B_MPC + obj.R_MPC);

        % Ensure that Hessian is symmetric (removes numerical errors)
        obj.H_MPC = (obj.H_MPC + obj.H_MPC.')/2;

        % Offset coefficient
        obj.fBar_MPC = 2*(obj.B_MPC.'*obj.Q_MPC + obj.N_MPC.')*obj.A_MPC;

        % Construct inequality constraint matrices for slew rate
        % limits (Slew Constraints). Assume u = 0 initially.
        W = eye(N_horizon) - diag(ones(N_horizon-1,1), -1);
        obj.CONST_INEQUAL_A = [W; -W];
        CONST_INEQUAL_A_TRANSPOSE = obj.CONST_INEQUAL_A';

        DeltaUmin = [0; zeros(N_horizon-1,1)] + ones(N_horizon,1)*obj.delta_u_min;
        DeltaUmax = [0; zeros(N_horizon-1,1)] + ones(N_horizon,1)*obj.delta_u_max;

        obj.CONST_INEQUAL_B = [DeltaUmax; -DeltaUmin];


        % Construct upper and lower bound constraint vectors
        obj.CONST_BOUND_UPPER = ones(obj.N_horizon,1)*obj.u_max;
        obj.CONST_BOUND_LOWER = ones(obj.N_horizon,1)*obj.u_min;

        
        % Initialise control signal to 0
        obj.u = 0;
        % Update indicator
        obj.controlMode = 'Regulation'; 
    end







    % MPC with reference feedforward and integral action
    function controlInit_reference_integrator(obj,A,B,Q,R,N,T,N_horizon,u_max,u_min,delta_u_max,delta_u_min,C_r,D_r,y_ref)
        % Store properties
        obj.N_horizon = N_horizon;
        obj.u_max = u_max;
        obj.u_min = u_min;
        obj.delta_u_max = delta_u_max;
        obj.delta_u_min = delta_u_min;
        obj.A = A;
        obj.B = B;
        obj.C_r = C_r;
        obj.D_r = D_r;
        obj.Q = Q;
        obj.R = R;
        obj.N = N;
        obj.T = T;
        obj.y_ref = y_ref;
        

        % Construct Nff vector and extract feed-forward gains Nx, Nu
        N_feedforward = (([obj.A  obj.B; obj.C_r  obj.D_r])^(-1))*([zeros(2,1); 1]);
        obj.N_x = N_feedforward(1:2);
        obj.N_u = N_feedforward(3);



        [Ad,Bd] = c2d(obj.A,obj.B,T);
        
        % Augment model with integrator
        Aid = [Ad zeros(2,1); T*obj.C_r 1];
        Bid = [Bd; obj.T*obj.D_r];


        Qd = Q;
        Rd = R;
        Nd = N;
        

        % Compute terminal cost Qf using 'dare' (or otherwise)
        [Qf,~,~] = dare(Aid,Bid,Qd,Rd,Nd,[]);

        % Construct concatinated matrices
        size_x = length(Aid);
        [~, size_u] = size(Bid);

        % Pre-allocate space for results
        obj.A_MPC = zeros(size_x*(N_horizon+1), size_x);
        obj.B_MPC = zeros(size_x*(N_horizon+1), N_horizon*size_u);
        obj.Q_MPC = zeros(size_x*(N_horizon+1), size_x*(N_horizon+1));
        obj.R_MPC = zeros(N_horizon*size_u, N_horizon*size_u);
        obj.N_MPC = zeros(size_x*(N_horizon+1), N_horizon*size_u);

  

        %% Construct matrices by looping through horizon
        obj.A_MPC(1:size_x,:) = eye(size_x);
        obj.Q_MPC(size_x*N_horizon+1:end, size_x*N_horizon+1:end) = Qf;

        for i=1:N_horizon
        obj.A_MPC(i*size_x+1:(i+1)*size_x,:) = Aid^i;
        obj.Q_MPC((i-1)*size_x+1:i*size_x, (i-1)*size_x+1:i*size_x) = Qd;
        obj.R_MPC(size_u*(i-1)+1:size_u*i, size_u*(i-1)+1:size_u*i) = Rd;
        obj.N_MPC((i-1)*size_x+1:i*size_x, size_u*(i-1)+1:size_u*i) = Nd;
        for j=1:i
        obj.B_MPC(i*size_x+1:(i+1)*size_x,size_u*(j-1)+1:size_u*j) = Aid^(i-j)*Bid;
        end
        end

        % Construct quadratic cost matrices
        % Hessian
        obj.H_MPC = 2*(obj.B_MPC.'*obj.Q_MPC*obj.B_MPC + obj.B_MPC.'*obj.N_MPC + obj.N_MPC.'*obj.B_MPC + obj.R_MPC);

        % Ensure that Hessian is symmetric (removes numerical errors)
        obj.H_MPC = (obj.H_MPC + obj.H_MPC.')/2;

        % Offset coefficient
        obj.fBar_MPC = 2*(obj.B_MPC.'*obj.Q_MPC + obj.N_MPC.')*obj.A_MPC;

        % Construct inequality constraint matrices for slew rate
        % limits (Slew Constraints). Assume u = 0 initially.
        W = eye(N_horizon) - [zeros(1,obj.N_horizon); eye(obj.N_horizon-1) zeros(N_horizon-1,1)];
        obj.CONST_INEQUAL_A = [W; -W];
        CONST_INEQUAL_A_TRANSPOSE = obj.CONST_INEQUAL_A';

        DeltaUmin = [0; zeros(N_horizon-1,1)] + ones(N_horizon,1)*obj.delta_u_min;
        DeltaUmax = [0; zeros(N_horizon-1,1)] + ones(N_horizon,1)*obj.delta_u_max;

        obj.CONST_INEQUAL_B = [DeltaUmax; -DeltaUmin];


        % Construct upper and lower bound constraint vectors
        obj.CONST_BOUND_UPPER = ones(obj.N_horizon,1)*obj.u_max;
        obj.CONST_BOUND_LOWER = ones(obj.N_horizon,1)*obj.u_min;

        
        % Initialise control signal to 0
        obj.u = 0;

        % Initialise integrator
        obj.z = 0;

        % Update indicator
        obj.controlMode = "Reference_integrator";
    end















    % Compute the control input for MPC
    function u = control(obj,x)
        % Switch between control modes
        switch obj.controlMode

            % Regulation only control
            case 'Regulation'
            % Update slew constraints based on previous input 'u'
            obj.CONST_INEQUAL_B(1) = obj.u(1) + obj.delta_u_max;
            obj.CONST_INEQUAL_B(obj.N_horizon+1) = -obj.u(1) - obj.delta_u_min;

            % Compute 'f' vector
            f_MPC = obj.fBar_MPC*x;

            % Run optimisation using quadprog to find optimal sequence of control
            % inputs
            U_MPC = quadprog(obj.H_MPC, f_MPC, obj.CONST_INEQUAL_A, obj.CONST_INEQUAL_B, [], [], obj.u_min, obj.u_max);

            % Extract and return the first control input
            [~, size_u] = size(obj.B);
            obj.u = U_MPC(1:size_u);
            u = obj.u;




            
        % Reference Feed-Forward with Integral Action
        case "Reference_integrator"
            
            % Compute feed-forward terms u_star and x_star
            x_star = obj.N_x*obj.y_ref;
            u_star = obj.N_u*obj.y_ref;

            xi_star = [x_star; 0];  % For integral action


            X_Star_MPC = repmat(xi_star,obj.N_horizon+1,1);
            U_Star_MPC = repmat(u_star,obj.N_horizon,1);

            % Update slew constraints based on previous input 'u'
            obj.CONST_INEQUAL_B(1) = obj.u(1) + obj.delta_u_max;
            obj.CONST_INEQUAL_B(obj.N_horizon+1) = -obj.u(1) - obj.delta_u_min;


            % Compute 'f' vector (Reference feedforward and integral
            % action)
            f_MPC = obj.fBar_MPC*[x; obj.z] - obj.fBar_MPC*xi_star - obj.H_MPC*U_Star_MPC;


            % Run optimisation using quadprog to find optimal sequence of control
            % inputs
            U_MPC = quadprog(obj.H_MPC, f_MPC, obj.CONST_INEQUAL_A, obj.CONST_INEQUAL_B, [], [], obj.u_min, obj.u_max);


            
            % Extract and return the first control input
            [~, size_u] = size(obj.B);
            obj.u = U_MPC(1:size_u);
            u = obj.u;


        % Update integrator state for next iteration
        obj.z = obj.z + obj.T*obj.C_r*(x - x_star) + obj.T*obj.D_r*(u - u_star); % We want to design a discrete time controller since the STM32 can only take samples at discrete time intervals. Hence why we use this equation




            % Default case if control is un-initialised
            otherwise
            u = 0;
        end
    end




    end
end