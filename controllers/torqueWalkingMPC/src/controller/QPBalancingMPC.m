% QPBALANCINGMPC this function configures a quadratic programming solver
%                (namely, qpOASES) which is used for optimizing the contact
%                forces together with internal toruqes while the robot is
%                performing a balancing tasks. The optimization procedure is
%                subject to both equality and inequality constraints.
% 
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model by means of S-function block.
%
% FORMAT: [] = QPBalancingMPC(block)        
%
% INPUT:  - tau_0 = [n * 1] initial joint torques 
%         - feetInContact = [2 * 1] feet contact status (0 not active, 1 active)
%         - Hessian = [n+6 * n+6] hessian matrix for QP solver
%         - gradient = [n+6 * 1] gradient for QP solver
%         - ConstraintMatrix_equality = [n_c * n+6] equality constraints matrix
%         - biasVectorConstraint_equality = [n_c * 1] equality constraints bias vector
%         - ConstraintMatrix_inequality = [n_c * 1] equality constraints matrix
%         - biasVectorConstraint_inequality = [n_c * n+6] equality constraints matrix
%
% OUTPUT: - f_LFoot = [6 * 1] left foot external wrenches
%         - f_RFoot = [6 * 1] right foot external wrenches
%         - tau = [n * 1] joint torques
%         - exitFlag = it returns 0 if the QP solver did not fail, a different 
%                      number otherwise
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function QPBalancingMPC(block)

    setup(block);
    
    function setup(block)
        
        block.NumInputPorts  = 8; 
        block.NumOutputPorts = 4;
        
        % Setup port properties to be dynamic
        block.SetPreCompInpPortInfoToDynamic;
        block.SetPreCompOutPortInfoToDynamic;
        
        % Output dimensions. Desired joint torques dimension is inherited
        % from S-Function block parameter (ROBOT_DOF)
        block.OutputPort(1).Dimensions = block.DialogPrm(1).Data; % tau
        block.OutputPort(2).Dimensions = 6;                       % f Left foot
        block.OutputPort(3).Dimensions = 6;                       % f Right foot     
        block.OutputPort(4).Dimensions = 1;                       % Exit flag QP  
        
        % Input format
        for i=1:block.NumInputPorts
            
            block.InputPort(i).DatatypeID  = -1; % inherited      
            block.InputPort(i).Complexity  = 'Real';
            block.InputPort(i).DirectFeedthrough = true;
        end

       % Output format 
       for i =1:block.NumOutputPorts
           
           block.OutputPort(i).DatatypeID  = 0; % double
           block.OutputPort(i).Complexity  = 'Real';
       end
       
       % Register parameters
       block.NumDialogPrms = 3;
       
       % Register sample times
       %
       %  [0 offset]            : Continuous sample time
       %  [positive_num offset] : Discrete sample time
       %  [-1, 0]               : Inherited sample time
       %  [-2, 0]               : Variable sample time
       %
       block.SampleTimes = [-1 0];
       
       % Specify the block simStateCompliance. The allowed values are:
       %
       %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
       %    'DefaultSimState', < Same sim state as a built-in block
       %    'HasNoSimState',   < No sim state
       %    'CustomSimState',  < Has GetSimState and SetSimState methods
       %    'DisallowSimState' < Error out when saving or restoring the model sim state
       block.SimStateCompliance = 'DefaultSimState';
         
       %% ----------------------------------------------------------------- 
       % The MATLAB S-function uses an internal registry for all
       % block methods. You should register all relevant methods
       % (optional and required) as illustrated below. You may choose
       % any suitable name for the methods and implement these methods
       % as local functions within the same file. See comments
       % provided for each function for more information.
       block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
       block.RegBlockMethod('Outputs', @Outputs); % Required
       block.RegBlockMethod('Terminate', @Terminate); % Required    
       block.RegBlockMethod('SetInputPortDimensions', @SetInputPortDimensions);
       block.RegBlockMethod('SetOutputPortDimensions',@SetOutputPortDimensions); 
       
    end

    % Registered methods
    function SetInputPortSamplingMode(block, idx, fd)
    
        block.InputPort(idx).SamplingMode = fd;

        for i=1:block.NumOutputPorts
            block.OutputPort(i).SamplingMode = fd;
        end
    end

    %% PostPropagationSetup:
    %%   Functionality    : Setup work areas and state variables. Can
    %%                      also register run-time methods here
    %%   Required         : No
    %%   C-Mex counterpart: mdlSetWorkWidths
    %%
    %% InitializeConditions:
    %%   Functionality    : Called at the start of simulation and if it is 
    %%                      present in an enabled subsystem configured to reset 
    %%                      states, it will be called when the enabled subsystem
    %%                      restarts execution to reset the states.
    %%   Required         : No
    %%   C-MEX counterpart: mdlInitializeConditions
    %%
    % function InitializeConditions(block)


    % end
    
    %% Start:
    %%   Functionality    : Called once at start of model execution. If you
    %%                      have states that should be initialized once, this 
    %%                      is the place to do it.
    %%   Required         : No
    %%   C-MEX counterpart: mdlStart
    %%
    % function Start(block)
 
        % block.Dwork(1).Data = 0;

    % end

    %% Outputs:
    %%   Functionality    : Called to generate block outputs in
    %%                      simulation step
    %%   Required         : Yes
    %%   C-MEX counterpart: mdlOutputs
    %%
    function Outputs(block)
   
        % block inputs
        tau_0                           = block.InputPort(1).Data;
        feetInContact                   = block.InputPort(2).Data;
        Hessian                         = block.InputPort(3).Data;
        gradient                        = block.InputPort(4).Data;
        ConstraintMatrix_equality       = block.InputPort(5).Data;
        biasVectorConstraint_equality   = block.InputPort(6).Data;
        ConstraintMatrix_inequality     = block.InputPort(7).Data;
        biasVectorConstraint_inequality = block.InputPort(8).Data;
        
        % block parameters
        ROBOT_DOF = block.DialogPrm(1).Data;
        Sat       = block.DialogPrm(2).Data;
        Config    = block.DialogPrm(3).Data;

        % initialize continuity constraint
        persistent tau_previousStep;
        
        if isempty(tau_previousStep)
            tau_previousStep = tau_0;
        end
    
        % What follows aims at defining the hessian matrix H, the bias
        % vector g, and the constraint matrix A for the formalism of qpOases,ie
        %
        %    min (1/2) x'*H*x + x'*g
        %     
        %       s.t. lbA < A*x < ubA
        %
        % For further information, see
        % 
        % http://www.coin-or.org/qpOASES/doc/3.0/manual.pdf
        %
    
        % CASE 1: left foot in contact
        
        if feetInContact(1) > (1-Sat.toll_feetInContact) && feetInContact(2) < Sat.toll_feetInContact
        
            % In this case, 
            % 
            % x = [f_LFoot
            %      tau]
            %
            H    = Hessian([1:6,13:end],[1:6,13:end]);
            g    = gradient([1:6,13:end]);

            % constraint matrix (contains both equality and inequality
            % constraints). Inequality constraints are reduced to the left
            % foot contact only.
            A    = [ConstraintMatrix_inequality(:,1:6), zeros(length(biasVectorConstraint_inequality),ROBOT_DOF);
                    ConstraintMatrix_equality(:,[1:6,13:end])];
              
        % CASE 2: right foot in contact
        
        elseif feetInContact(1) < Sat.toll_feetInContact && feetInContact(2) > (1-Sat.toll_feetInContact)
            
            % In this case, 
            % 
            % x = [f_RFoot
            %      tau]
            %
            H    = Hessian(7:end,7:end);
            g    = gradient(7:end);

            % constraint matrix (contains both equality and inequality
            % constraints). Inequality constraints are reduced to the right
            % foot contact only.
            A    = [ConstraintMatrix_inequality(:,7:end), zeros(length(biasVectorConstraint_inequality),ROBOT_DOF);
                    ConstraintMatrix_equality(:,7:end)];
                    
        % CASE 3: both feet in contact
        
        else
            
            % In this case, 
            % 
            % x = [f_LFoot
            %      f_RFoot
            %      tau]
            %         
            H    = Hessian;
            g    = gradient;

            % constraint matrix (contains both equality and inequality
            % constraints).
            A    =  [ConstraintMatrix_inequality, zeros(length(biasVectorConstraint_inequality),ROBOT_DOF);
                     ConstraintMatrix_equality];    
        end
        
        % to avoid the equality constraints to be unfeasible for numerical 
        % errors, a small tolerance is added to the bias vectors
       	eps = 0.0001*ones(size(biasVectorConstraint_equality,1),1);
                
        % upper bound constraints    
        ubA  = [biasVectorConstraint_inequality;
                (biasVectorConstraint_equality+eps)];
                
        % lower bound constraints. Note that equality constraints are
        % expressed in the form of: b_eq < A*x < b_eq
        lbA  = [-1e14*ones(length(biasVectorConstraint_inequality),1);
                 (biasVectorConstraint_equality-eps)];
            
        % Use continuity constraint. In case this option is selected, an
        % additional constraint is applied to the optimization procedure.
        % In particular, the numerical derivative of the torque is
        % calculated as follows:
        %
        %    tauDot = (tau(k)-tau(k-1))/dt. [1]
        %
        % Then, one can imagine to fix a limit for the torque derivative
        % tauDot. In this case, one may think of apply a further constraint:
        %
        %    tau(k)_max =  tauDot_max*dt + tau(k-1)  [2]
        %    tau(k)_min = -tauDot_max*dt + tau(k-1)  [2]
        %    
        %    tau(k)_min < tau(k) < tau(k)_max       [3] 
        %
        if Config.QP_USE_CONTINUITY_CONSTRAINTS && ~isempty(tau_previousStep)
        
            A    = [A; 
                    zeros(ROBOT_DOF,size(A,2)-ROBOT_DOF), eye(ROBOT_DOF)];
            ubA  = [ubA;
                    Sat.tauDot_max*Config.t_step+tau_previousStep];
            lbA  = [lbA;
                   -Sat.tauDot_max*Config.t_step+tau_previousStep];
        end
        
        % Enforce symmetry of the Hessian matrix. Given the Hessian matrix
        % is symmetrix, and anyway the term x'*H*x is 0 if H is a
        % skew-symmetric matrix, we enforce the symmetry of H
        %
        H = 0.5*(H + transpose(H));
      
        %% ----------------------------------------------------------------
        %% QP optimization procedure using qpOASES
        %% ----------------------------------------------------------------
        [u,~,exitFlagQP,~,~,~] = qpOASES(H,g,A,[],[],lbA,ubA);     
    
        % separate joint torques (actual control input) from contact forces 
        % for all diffrent cases
        if feetInContact(1) > (1-Sat.toll_feetInContact) && feetInContact(2) < Sat.toll_feetInContact
            
            % left foot balancing
            f_LFoot = u(1:6);
            f_RFoot = zeros(6,1);
            tau     = u(7:end);
            
        elseif feetInContact(1) < Sat.toll_feetInContact && feetInContact(2) > (1-Sat.toll_feetInContact)
            
            % right foot balancing
            f_LFoot = zeros(6,1);
            f_RFoot = u(1:6);
            tau     = u(7:end);
            
        else            
            % two feet balancing
            f_LFoot = u(1:6);
            f_RFoot = u(7:12);
            tau     = u(13:end);   
        end
        
        % in case of QP errors, apply the previous joint torque values
        if exitFlagQP ~= 0 
        
            tau = tau_previousStep;
        else
            tau_previousStep = tau;
        end

        %% Outputs
        block.OutputPort(1).Data = tau;
        block.OutputPort(2).Data = f_LFoot;
        block.OutputPort(3).Data = f_RFoot; 
        block.OutputPort(4).Data = exitFlagQP;
    
    end

    function Terminate(~)

    end

    function SetOutputPortDimensions(s, port, dimsInfo)
        s.OutputPort(port).Dimensions = dimsInfo;
    end
    
    function SetInputPortDimensions(s, port, dimsInfo)
        s.InputPort(port).Dimensions = dimsInfo;
    end
end
