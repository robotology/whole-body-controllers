% QPINVERSEKINEMATICS this function configures a quadratic programming solver
%                     (namely, qpOASES) which minimizes the error between
%                     the state accelerations and their desired values,
%                     while a set of desired Cartesian accelerations is
%                     considered as a constraint.
% 
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model by means of S-function block.
%
% FORMAT: [] = QPInverseKinematics(block)        
%
% INPUT:  - Hessian = [ROBOT_DOF + 6 * ROBOT_DOF + 6] Hessian matrix 
%         - gradient = [ROBOT_DOF + 6 * 1] gradient for QP optimization
%         - ConstraintMatrix = [ n_c * ROBOT_DOF + 6] constraint matrix
%         - biasVectorConstraint = [ n_c * 1] bias vector constraints
% 
% OUTPUT: - nuDot = [ROBOT_DOF + 6 * 1] state accelerations
%         - exitFlagQP = a flag for verifying eventual QP failures
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function QPInverseKinematics(block)

    setup(block);
    
    function setup(block)
        
        block.NumInputPorts  = 4; 
        block.NumOutputPorts = 2;
        
        % Setup port properties to be dynamic
        block.SetPreCompInpPortInfoToDynamic;
        block.SetPreCompOutPortInfoToDynamic;
        
        % Output dimensions. Desired state dimension is inherited
        % from S-Function block parameter (ROBOT_DOF)
        block.OutputPort(1).Dimensions = block.DialogPrm(1).Data + 6; % state accelerations    
        block.OutputPort(2).Dimensions = 1;                           % Exit flag QP  
        
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
       block.NumDialogPrms     = 3;
       
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
        Hessian              = block.InputPort(1).Data;
        gradient             = block.InputPort(2).Data;
        ConstraintMatrix     = block.InputPort(3).Data;
        biasVectorConstraint = block.InputPort(4).Data;

        % block parameters
        ROBOT_DOF = block.DialogPrm(1).Data;
        Sat       = block.DialogPrm(2).Data;
        Config    = block.DialogPrm(3).Data;

        % initialize continuity constraint
        persistent nuDot_previousStep;
        
        if isempty(nuDot_previousStep)
            nuDot_previousStep = zeros(ROBOT_DOF + 6,1);
        end
    
        % define Hessian, gradient and constraints
        A    = ConstraintMatrix;
        H    = (Hessian + transpose(Hessian))*0.5;
        g    = gradient;
        
        % to avoid the equality constraints to be unfeasible for numerical errors,
        % a small tolerance is added to the bias vectors
        eps      = [0.001*ones(3,1); 0.1*ones(3,1); ...
                    0.001*ones(3,1); 0.1*ones(3,1); ...
                    0.001*ones(3,1); 0.1*ones(3,1)];
                    
        ubA  = biasVectorConstraint+eps;
        lbA  = biasVectorConstraint-eps;
        
        % Use continuity constraint. In case this option is selected, an
        % additional constraint is applied to the optimization procedure.
        % In particular, the numerical derivative of the state acceleration is
        % calculated as follows:
        %
        %    nuDDot = (nuDot(k)-nuDot(k-1))/dt. [1]
        %
        % Then, one can imagine to fix a limit for the state acc. derivative
        % nuDDot. In this case, one may think of apply a further constraint:
        %
        %    nuDot(k)_max =  nuDDot_max*dt + nuDot(k-1)  [2]
        %    nuDot(k)_min = -nuDDot_max*dt + nuDot(k-1)  [2]
        %    
        %    nuDot(k)_min < nuDot(k) < nuDot(k)_max      [3] 
        %
        if Config.QP_IKIN_USE_CONTINUITY_CONSTRAINTS && ~isempty(nuDot_previousStep)
        
            A    = [A; 
                    eye(ROBOT_DOF + 6)];
            ubA  = [ubA;
                    Sat.nuDDot_max*Config.t_step+nuDot_previousStep];
            lbA  = [lbA;
                   -Sat.nuDDot_max*Config.t_step+nuDot_previousStep];
        end
     
        %% ------------------------------------------------------------- %%
        %% QP optimization procedure using qpOASES 
        %% ------------------------------------------------------------- %%
        [nuDot,~,exitFlagQP,~,~,~] = qpOASES(H,g,A,[],[],lbA,ubA);     
    
        % in case of QP errors, do not move! this implies to send zero
        % accelerations as reference
        if exitFlagQP ~= 0
 
            nuDot = zeros(ROBOT_DOF+6, 1);
        else
            nuDot_previousStep = nuDot;
        end

        %% Outputs
        block.OutputPort(1).Data = nuDot;
        block.OutputPort(2).Data = exitFlagQP;
    
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
