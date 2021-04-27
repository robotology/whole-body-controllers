% CONFIGSTATEMACHINE configures the state machine (type of demo, velocity
%                    of the demo, repeat movements, and so on).

%% --- Initialization ---

% If true, the robot CoM will follow a desired reference trajectory (COORDINATOR DEMO ONLY)
Config.LEFT_RIGHT_MOVEMENTS = false;

% If equal to one, the desired values of the center of mass are smoothed internally 
Config.SMOOTH_COM_DES       = true;   

% If equal to one, the desired values of the postural tasks are smoothed internally 
Config.SMOOTH_JOINT_DES     = true;   

% Joint torques saturation [Nm]
Sat.torque                  = 60;

% Joint torques rate of change saturation
Sat.uDotMax                 = 300;

% max unsigned difference between two consecutive (measured) joint positions, 
% i.e. delta_qj = abs(qj(k) - qj(k-1))
Sat.maxJointsPositionDelta  = 15*pi/180; % [rad] 

%% Regularization parameters
Reg.pinvDamp_baseVel        = 1e-7;
Reg.pinvDamp                = 1; 
Reg.pinvTol                 = 1e-5;
Reg.KP_postural             = 0.1;
Reg.KD_postural             = 0;
Reg.HessianQP               = 1e-7;    

%% State Machine configuration

% time between two yoga positions
StateMachine.joints_pauseBetweenYogaMoves = 5;

% contact forces threshold
StateMachine.wrench_thresholdContactOn    = 50;
StateMachine.wrench_thresholdContactOff   = 100;

% threshold on CoM and joints error
StateMachine.CoM_threshold                = 0.01; 
StateMachine.joints_thresholdNotInContact = 5;
StateMachine.joints_thresholdInContact    = 50;

% initial state for state machine
StateMachine.initialState                 = 1;

% other configuration parameters for state machine
StateMachine.tBalancing                   = 1;
StateMachine.tBalancingBeforeYoga         = 1;
StateMachine.yogaExtended                 = true;
StateMachine.skipYoga                     = false;
StateMachine.demoOnlyBalancing            = false;
StateMachine.demoStartsOnRightSupport     = false; % If false, the Yoga demo is performed on the left foot first
StateMachine.yogaAlsoOnRightFoot          = true; % TO DO: yoga on both feet starting from right foot (not available for now)

%%%% List of possible "Yoga in loop" %%%%

% the robot will repeat the FULL DEMO (two feet balancing, yoga on left
% foot, back on two feet, yoga right foot, back on two feet). The demo is
% repeated until the user stops the Simulink model. This option is ignored
% if Sm.demoStartsOnRightSupport = true.
StateMachine.twoFeetYogaInLoop            = false;

% the robot will repeat the ONE FOOT yoga for the number of times the user
% specifies in the Sm.yogaCounter option. The robot WILL NOT go back to two
% feet balancing in between to consecutive yoga. WARNING: if the option 
% Sm.yogaAlsoOnRightFoot is true, then the robot will repeat first the yoga
% on left foot for the number of times the user specifies in the Sm.yogaCounter,
% and then it will repeat the yoga on the right foot for the same number of times.
StateMachine.oneFootYogaInLoop            = false;
StateMachine.yogaCounter                  = 5;

%% Parameters for motors reflected inertia

% transmission ratio (1/N)
Config.Gamma                 = 0.01*eye(ROBOT_DOF);

% modify the value of the transmission ratio for the hip pitch. 
% TODO: avoid to hard-code the joint numbering
Config.Gamma(end-5, end-5)   = 0.0067;
Config.Gamma(end-11,end-11)  = 0.0067;

% motors inertia (Kg*m^2)
legsMotors_I_m               = 0.0827*1e-4;
torsoPitchRollMotors_I_m     = 0.0827*1e-4;
torsoYawMotors_I_m           = 0.0585*1e-4;
armsMotors_I_m               = 0.0585*1e-4;

% add harmonic drives reflected inertia
if Config.INCLUDE_HARMONIC_DRIVE_INERTIA
   
    legsMotors_I_m           = legsMotors_I_m + 0.054*1e-4;
    torsoPitchRollMotors_I_m = torsoPitchRollMotors_I_m + 0.054*1e-4;
    torsoYawMotors_I_m       = torsoYawMotors_I_m + 0.021*1e-4;
    armsMotors_I_m           = armsMotors_I_m + 0.021*1e-4; 
end
 
Config.I_m                   = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                     torsoYawMotors_I_m;
                                     armsMotors_I_m*ones(8,1);
                                     legsMotors_I_m*ones(12,1)]);

% parameters for coupling matrices. Updated according to the wiki:
%
% http://wiki.icub.org/wiki/ICub_coupled_joints 
%
% and corrected according to https://github.com/robotology/robots-configuration/issues/39
t            = 0.615;
r            = 0.022;
R            = 0.04;

% coupling matrices
T_LShoulder  = [-1  0  0;
                -1 -t  0;
                 0  t -t];

T_RShoulder  = [ 1  0  0;
                 1  t  0;
                 0 -t  t];

T_torso      = [ 0.5    -0.5     0;
                 0.5     0.5     0;
                 r/(2*R) r/(2*R) r/R];
       
if Config.INCLUDE_COUPLING
       
    Config.T = blkdiag(T_torso,T_LShoulder,1,T_RShoulder,1,eye(12));
else          
    Config.T = eye(ROBOT_DOF);
end

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff  = 0;

% Config.USE_DES_JOINT_ACC_FOR_MOTORS_INERTIA if true, the desired joints
% accelerations are used for computing the feedforward term in joint
% torques calculations. Not effective if Config.K_ff = 0.
Config.USE_DES_JOINT_ACC_FOR_MOTORS_INERTIA = false;

%% Constraints for QP for balancing

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the circle 
% in each cicle's quadrant
numberOfPoints               = 4;  
forceFrictionCoefficient     = 1/3;    
torsionalFrictionCoefficient = 1/75;
fZmin                        = 10;

% physical size of the foot                             
feet_size                    = [-0.07  0.12 ;    % xMin, xMax
                                -0.045 0.05 ];   % yMin, yMax  
                                                    
% Compute contact constraints (friction cone, unilateral constraints)
[ConstraintsMatrix, bVectorConstraints] = wbc.computeRigidContactConstraints ...
    (forceFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient, feet_size, fZmin);