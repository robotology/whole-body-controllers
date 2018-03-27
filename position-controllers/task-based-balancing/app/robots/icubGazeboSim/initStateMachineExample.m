% INITSTATEMACHINEEXAMPLE initializes the robot configuration for running
%                        'EXAMPLE_STATEMACHINE' demo. 
%
% USAGE: please note that this function is automatically executed when
%        running the Simulink model.
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---

% SIMULATION SETUP
%
% Frames name list
Frames.IMU = 'imu_frame';
Frames.COM = 'com';
Frames.BASE_LINK = 'root_link';
Frames.LEFT_FOOT = 'l_sole';
Frames.RIGHT_FOOT = 'r_sole';
Frames.ROT_TASK_LINK = 'neck_2';

% Emergency stop if MPC ports are streaming null data (MPC_WALKING DEMO ONLY)
Config.CHECK_PORTS_WALKING = false;
Config.WALKING_WITH_MPC = false;

% If true, joint references are modified in order not to be in conflict with
% the Cartesian tasks. The new joint references are calculated by means of an
% integration based inverse kinematics
Config.USE_INVERSE_KINEMATICS = true;

% If true, the output of QP solver will be forced to be continuous
Config.QP_USE_CONTINUITY_CONSTRAINTS = true;
Config.QP_IKIN_USE_CONTINUITY_CONSTRAINTS = true;

% If true, the IMU orientation is used in order to estimate the
% base-to-world transformation matrix
Config.USE_IMU4EST_BASE = false;

% If true, the orientation provided by the IMU is corrected using the neck
% positions (requires the neck port to be active)
Config.CORRECT_IMU_WITH_NECK_POS = true;

% If true, IMU pitch and yaw are not considered for estimating base-to-world transform
Config.FILTER_IMU_YAW = false;
Config.FILTER_IMU_PITCH = false;

% True if left foot is initially in contact with the ground (if false,
% right foot is assumed to be in contact) (EXAMPLE_STATEMACHINE DEMO ONLY)
Config.LFoot_in_contact_at0 = false;

% If true, the robot will just balance on two feet (EXAMPLE_STATEMACHINE DEMO ONLY)
Config.ONLY_BALANCING = false;

% If Config.ONLY_BALANCING = false, this is the time the robot will balance
% before it starts moving (EXAMPLE_STATEMACHINE DEMO ONLY)
Config.t_balancing = 2;

% If true, simulation is stopped when qpOASES outputs a "-2" error (QP is unfeasible)
Config.CHECK_QP_ERROR = true;

%% Robot setup 

% Joint torque saturation
Sat.tau_max = 60; % [Nm]

% Saturation on torque derivative (for QP solver)
Sat.tauDot_max = 10000;

% Saturation on state jerk (for QP based inverse kinematics)
Sat.nuDDot_max = 10000;

% Weight for the joint minimization task
Sat.weight_tau = 0.1;

% Numerical tolerance for assuming a foot on contact
Sat.toll_feetInContact = 0.1;

% Damping for the pseudoinverse used for computing the floating base velocity
Sat.pinvDamp_nu_b = 1e-6;

% If true, the feet accelerations are zero when the foot is in contact. If false, 
% feet accelerations are equal to a feedforward + feedback terms
Sat.zeroAccWhenFeetInContact = false;

%% Parameters for motors reflected inertia

% inverse of the transmission ratio
Config.invGamma = 100*eye(ROBOT_DOF);
% torso yaw has a bigger reduction ratio
Config.invGamma(3,3) = 200;

% motors inertia (Kg*m^2)
legsMotors_I_m           = 0.0827*1e-4;
torsoPitchRollMotors_I_m = 0.0827*1e-4;
torsoYawMotors_I_m       = 0.0585*1e-4;
armsMotors_I_m           = 0.0585*1e-4;
Config.I_m               = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                 torsoYawMotors_I_m;
                                 armsMotors_I_m*ones(8,1);
                                 legsMotors_I_m*ones(12,1)]);

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff     = 0;

%% Smoothing of reference trajectories

% If true, reference trajectories are smoothed internally
Config.SMOOTH_COM_REF      = true;
Config.SMOOTH_LFOOT_POS    = true;
Config.SMOOTH_RFOOT_POS    = true;
Config.SMOOTH_LFOOT_ORIENT = true; 
Config.SMOOTH_RFOOT_ORIENT = true; 
Config.SMOOTH_ROT_TASK_REF = true;
Config.SMOOTH_JOINT_REF    = true; 

% Smoothing time for tasks and joints references [s]
Config.smoothingTime_CoM    = [2;2.5;2;2;1.5];
Config.smoothingTime_LFoot  = [1;1.5;2;2;2.5];
Config.smoothingTime_RFoot  = [2;1;1.5;2;3];
Config.smoothingTime_joints = [2;2;1;1.5;2];

% Gains that will influence the smoothing of reference orientations. The
% higher, the faster. Only positive or null values.
Config.LFoot_Kp_smoothing    = [1.5;1;1;1;2];
Config.LFoot_Kd_smoothing    = [1;1;2.5;1;2];
Config.RFoot_Kp_smoothing    = [1;1;2.5;1;2];
Config.RFoot_Kd_smoothing    = [1;1.5;1;1;2];
Config.rot_task_Kp_smoothing = [1;2;1.5;2;2.5];
Config.rot_task_Kd_smoothing = [1;2;1;1.5;2];

% Smoothing time for gain scheduling [s].
Config.smoothingTimeGains    = [1;2;3;1.5;2];

% Minimum value of the vertical force at contact location for the contact
% to be considered as active (MPC_WALKING DEMO ONLY)
Config.threshold_contact_activation = 2.5; % [N]

%% CoM and feet references (EXAMPLE_STATEMACHINE DEMO ONLY)

% add a delta to the right foot position. 
%
% dimension: [m]
% format: [x;y;z]
%
Config.deltaPos_RFoot = [ 0.000 0.00  0.000; ...   % state = 1 two feet balancing
                          0.000 0.00  0.000; ...   % state = 2 move CoM on left foot
                         -0.025 0.00  0.025; ...   % state = 3 left foot balancing
                          0.000 0.00  0.000; ...   % state = 4 prepare for switching
                          0.000 0.00  0.000];      % state = 5 two feet balancing
   
%% Gains matrices

% CoM position and velocity gains
Gains.Kp_CoM = [50, 50, 50; ...  % state = 1 two feet balancing
                40, 40, 40; ...  % state = 2 move CoM on left foot
                55, 55, 55; ...  % state = 3 left foot balancing
                50, 50, 50; ...  % state = 4 prepare for switching
                45, 45, 45];     % state = 5 two feet balancing
                
Gains.Kd_CoM = 2*sqrt(Gains.Kp_CoM);

% Feet position and velocity gains
Gains.Kp_LFoot = [50, 50, 50, 30, 30, 30; ... % state = 1 two feet balancing
                  40, 40, 40, 40, 40, 40; ... % state = 2 move CoM on left foot
                  60, 60, 60, 45, 45, 45; ... % state = 3 left foot balancing
                  55, 55, 55, 45, 45, 45; ... % state = 4 prepare for switching
                  60, 60, 60, 45, 45, 45];    % state = 5 two feet balancing
              
Gains.Kd_LFoot = 2*sqrt(Gains.Kp_LFoot);

Gains.Kp_RFoot = [50, 50, 50, 30, 30, 30; ... % state = 1 two feet balancing
                  40, 40, 40, 40, 40, 40; ... % state = 2 move CoM on left foot
                  60, 60, 60, 45, 45, 45; ... % state = 3 left foot balancing
                  55, 55, 55, 45, 45, 45; ... % state = 4 prepare for switching
                  60, 60, 60, 45, 45, 45];    % state = 5 two feet balancing

Gains.Kd_RFoot = 2*sqrt(Gains.Kp_RFoot); 

% Root link orientation and angular velocity gains
Gains.Kp_rot_task = [20, 20, 20; ...  % state = 1 two feet balancing
                     40, 40, 40; ...  % state = 2 move CoM on left foot
                     25, 25, 25; ...  % state = 3 left foot balancing
                     30, 30, 30; ...  % state = 4 prepare for switching
                     20, 20, 20];     % state = 5 two feet balancing
                 
Gains.Kd_rot_task =  2*sqrt(Gains.Kp_rot_task); 

% Joint position and velocity gains
    
                    % torso      % left arm       % right arm      % left leg               % right leg                                   
Gains.impedances = [20  20  20,  10  10  10  10,  10  10  10  10,  30  35  40  60  10  20,  30  35  40  60  10  20;  ... % state = 1 two feet balancing          
                    30  30  30,  20  20  20  20,  20  20  20  20,  35  40  35  65  10  15,  35  40  35  65  10  15;  ... % state = 2 move CoM on left foot
                    25  25  25,  10  10  10  10,  10  10  10  10,  40  35  30  70  20  10,  40  35  30  70  20  10;  ... % state = 3 left foot balancing
                    35  35  35,  15  15  15  15,  15  15  15  15,  25  30  25  55  25  25,  25  30  25  55  25  25;  ... % state = 4 prepare for switching
                    20  20  20,  10  10  10  10,  10  10  10  10,  30  35  40  60  10  20,  30  35  40  60  10  20];     % state = 5 two feet balancing
                     
Gains.dampings   = zeros(size(Gains.impedances));

% Joints position and velocity gains for inverse kinematics
Gains.ikin_impedances = Gains.impedances(1,:);
Gains.ikin_dampings   = 2*sqrt(Gains.ikin_impedances); 
    
%% Constraints for QP for balancing - friction cone - z-moment - in terms of f

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the 
% circle in each cicle's quadrant 
numberOfPoints               = 4; 
forceFrictionCoefficient     = 1;  
torsionalFrictionCoefficient = 1/75;
fZmin                        = 10; % Min vertical force [N]

% Size of the foot
Config.footSize              = [-0.05  0.10;     % xMin, xMax
                                -0.025 0.025];   % yMin, yMax 
                            