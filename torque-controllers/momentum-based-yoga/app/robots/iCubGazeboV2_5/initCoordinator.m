% INITCOORDINATOR initializes the robot configuration for running
%                'COORDINATOR' demo. 
%

%% --- Initialization ---

% Feet in contact (COORDINATOR DEMO ONLY)
Config.LEFT_RIGHT_FOOT_IN_CONTACT = [1 1];

% Initial foot on ground. If false, right foot is used as default contact
% frame (this does not means that the other foot cannot be in contact too).
% (COORDINATOR DEMO ONLY)
Config.LEFT_FOOT_IN_CONTACT_AT_0 = true;

% If true, the robot CoM will follow a desired reference trajectory (COORDINATOR DEMO ONLY)
Config.DEMO_MOVEMENTS = false;

% If equal to true, the desired streamed values of the center of mass 
% are smoothed internally 
Config.SMOOTH_COM_DES = false;   

% If equal to true, the desired streamed values of the postural tasks are
% smoothed internally 
Config.SMOOTH_JOINT_DES = false;   

% torque saturation
Sat.torque = 34; 

% torque derivative max absolute value
Config.tauDot_maxAbs = 300;

%% Control gains

% PARAMETERS FOR TWO FEET BALANCING
if sum(Config.LEFT_RIGHT_FOOT_IN_CONTACT) == 2
    
    Gain.KP_COM             = diag([50 100 5]);
    Gain.KD_COM             = 2*sqrt(Gain.KP_COM)*0;
    Gain.KP_AngularMomentum = 5;
    Gain.KD_AngularMomentum = 2*sqrt(Gain.KP_AngularMomentum);

    % Impedances acting in the null space of the desired contact forces 
    impTorso            = [10   10   20]; 

    impArms             = [10   10   10    8];

    impLeftLeg          = [30   30   30    60   10   10]; 

    impRightLeg         = [30   30   30    60   10   10];                                            
end

% PARAMETERS FOR ONE FOOT BALANCING
if sum(Config.LEFT_RIGHT_FOOT_IN_CONTACT) == 1
    
    Gain.KP_COM               = diag([50  100  50]);
    Gain.KD_COM               = diag([0   0    0]);
    Gain.KP_AngularMomentum   = 1 ;
    Gain.KD_AngularMomentum   = 1 ;

    % Impedances acting in the null space of the desired contact forces    
    impTorso            = [20   20   30];
    
    impArms             = [15   15   15    8];
                        
    impLeftLeg          = [30   30   30    120   10   10];

    impRightLeg         = [30   30   30    60    10   10];   
end

Gain.impedances         = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
Gain.dampings           = 0*sqrt(Gain.impedances);

if (size(Gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end

% Smoothing time gain scheduling (YOGA DEMO ONLY)
Gain.SmoothingTimeGainScheduling = 2;

%% Parameters for motors reflected inertia

% transmission ratio
Config.Gamma = 0.01*eye(ROBOT_DOF);

% motors inertia (Kg*m^2)
legsMotors_I_m           = 0.0827*1e-4;
torsoPitchRollMotors_I_m = 0.0827*1e-4;
torsoYawMotors_I_m       = 0.0585*1e-4;
armsMotors_I_m           = 0.0585*1e-4;
Config.I_m               = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                 torsoYawMotors_I_m;
                                 armsMotors_I_m*ones(8,1);
                                 legsMotors_I_m*ones(12,1)]);

% parameters for coupling matrices                            
t  = 0.625;
r  = 0.022;
R  = 0.04;

% coupling matrices
T_LShoulder = [-1  0  0;
               -1 -t  0;
                0  t -t];

T_RShoulder = [ 1  0  0;
                1  t  0;
                0 -t  t];

T_torso = [0   -0.5     0.5;
           0    0.5     0.5;
           r/R  r/(2*R) r/(2*R)];
       
if Config.INCLUDE_COUPLING
       
    Config.T = blkdiag(T_torso,T_LShoulder,1,T_RShoulder,1,eye(12));
else          
    Config.T = eye(ROBOT_DOF);
end

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff  = 0;

%% References for CoM trajectory (COORDINATOR DEMO ONLY)

% that the robot waits before starting the left-and-right 
Config.noOscillationTime = 0;   

if Config.DEMO_MOVEMENTS && sum(Config.LEFT_RIGHT_FOOT_IN_CONTACT) == 2
        
    Config.directionOfOscillation = [0;1;0];
    % amplitude of oscillations in meters
    Config.amplitudeOfOscillation = 0.02;
    % frequency of oscillations in hertz
    Config.frequencyOfOscillation = 1;
else
    Config.directionOfOscillation  = [0;0;0];
    Config.amplitudeOfOscillation  = 0.0;  
    Config.frequencyOfOscillation  = 0.0;
end

%% State machine parameters

% smoothing time for joints and CoM
Sm.smoothingTimeCoM_Joints = 3; 

% time between two yoga positions (YOGA DEMO ONLY)
Sm.joints_pauseBetweenYogaMoves = 0;

% contact forces threshold (YOGA DEMO ONLY)
Sm.wrench_thresholdContactOn  = 1;
Sm.wrench_thresholdContactOff = 1;

% threshold on CoM and joints error (YOGA DEMO ONLY)
Sm.CoM_threshold                = 0; 
Sm.joints_thresholdNotInContact = 0;
Sm.joints_thresholdInContact    = 0;

% initial state for state machine (YOGA DEMO ONLY)
Sm.stateAt0 = 1;

% delta to be summed to the reference CoM position (YOGA DEMO ONLY)
Sm.CoM_delta = [0; 0; 0];

% joint references (YOGA DEMO ONLY)
Sm.joints_references   = zeros(1,ROBOT_DOF);
Sm.joints_leftYogaRef  = zeros(1,ROBOT_DOF+1);
Sm.joints_rightYogaRef = zeros(1,ROBOT_DOF+1);

% configuration parameters for state machine (YOGA DEMO ONLY) 
Sm.tBalancing               = 1;
Sm.tBalancingBeforeYoga     = 1;
Sm.skipYoga                 = false;
Sm.demoOnlyBalancing        = false;
Sm.demoStartsOnRightSupport = false;
Sm.yogaAlsoOnRightFoot      = false;
Sm.yogaInLoop               = false;

%% Constraints for QP for balancing

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the circle 
% in each cicle's quadrant
numberOfPoints               = 4;  
forceFrictionCoefficient     = 1/3;  
torsionalFrictionCoefficient = 1/75;

% physical size of the foot                             
feet_size                    = [-0.07 0.12;     % xMin, xMax
                                -0.04 0.04 ];   % yMin, yMax    
 
fZmin                        = 10;

%% Regularization parameters
Reg.pinvDamp_nu_b = 1e-7;
Reg.pinvDamp      = 2; 
Reg.pinvTol       = 1e-5;
Reg.impedances    = 0.1;
Reg.dampings      = 0;
Reg.HessianQP     = 1e-5;
