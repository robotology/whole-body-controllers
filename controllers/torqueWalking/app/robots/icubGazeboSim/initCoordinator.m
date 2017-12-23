% INITCOORDINATOR initializes the robot configuration for running
%                 'COORDINATOR' demo. 
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

% Joint torque saturation
Sat.tau_max = 60; % [Nm]

% Saturation on torque derivative (for QP solver)
Sat.tauDot_max = 10000;

% Saturation on state jerk (for QP based inverse kinematics)
Sat.nuDDot_max = 10000;

% Weight for the joint minimization task
Sat.weight_tau = 0.01;

% Numerical tolerance for assuming a foot on contact
Sat.toll_feetInContact = 0.1;

% Damping for the pseudoinverse used for computing the floating base velocity
Sat.pinvDamp_nu_b = 1e-6;

% True if left foot is initially in contact with the ground (if false,
% right foot is assumed to be in contact)
Config.LFoot_in_contact_at0 = true;

%% Smoothing of reference trajectories

% If true, reference trajectories are smoothed internally
Config.SMOOTH_COM_REF      = true;
Config.SMOOTH_LFOOT_POS    = true;
Config.SMOOTH_RFOOT_POS    = true;
Config.SMOOTH_LFOOT_ORIENT = true; 
Config.SMOOTH_RFOOT_ORIENT = true; 
Config.SMOOTH_ROT_TASK_REF = true;
Config.SMOOTH_JOINT_REF    = true; 

% Smoothing time for tasks and joints references [s].
Config.smoothingTime_CoM    = 2;
Config.smoothingTime_LFoot  = 2;
Config.smoothingTime_RFoot  = 2;
Config.smoothingTime_joints = 2;

% Gains that will influence the smoothing of reference orientations. The
% higher, the faster. Only positive or null values.
Config.LFoot_Kp_smoothing    = 1;
Config.LFoot_Kd_smoothing    = 1;
Config.RFoot_Kp_smoothing    = 1;
Config.RFoot_Kd_smoothing    = 1;
Config.rot_task_Kp_smoothing = 1;
Config.rot_task_Kd_smoothing = 1;

% Smoothing time for gain scheduling [s].
Config.smoothingTimeGains    = 1;

%% CoM references
if Config.DEMO_MOVEMENTS 

    Config.noOscillationTime       =  0;
    Config.directionOfOscillation  = [0;1;0];  % [x; y; z]
    Config.amplitudeOfOscillation  =  0.02;    % amplitude of oscillations in meters 
    Config.frequencyOfOscillation  =  0.2;     % frequency of oscillations in hertz 
else
    Config.noOscillationTime       =  0;
    Config.directionOfOscillation  = [0;0;0];
    Config.amplitudeOfOscillation  =  0.0; 
    Config.frequencyOfOscillation  =  0.0;
end
   
%% Gains matrices

% CoM position and velocity gains
Gains.Kp_CoM = [50, 50, 50];
Gains.Kd_CoM = 2*sqrt(Gains.Kp_CoM);

% Feet position and velocity gains
Gains.Kp_LFoot = [50, 50, 50, 20, 20, 20];
Gains.Kd_LFoot = 2*sqrt(Gains.Kp_LFoot);

Gains.Kp_RFoot = [50, 50, 50, 20, 20, 20];
Gains.Kd_RFoot = 2*sqrt(Gains.Kp_RFoot); 

% Root link orientation and angular velocity gains
Gains.Kp_rot_task = [20, 20, 20]; 
Gains.Kd_rot_task =  2*sqrt(Gains.Kp_rot_task); 

% Joint position and velocity gains
Gains.impedances = [20  20  20 ...              % torso 
                    10  10  10  8 ...           % left arm
                    10  10  10  8 ...           % right arm
                    30  30  30  60  10  10  ... % left leg
                    30  30  30  60  10  10];    % right leg

Gains.dampings   = zeros(1, ROBOT_DOF);

% Joints position and velocity gains for inverse kinematics
Gains.ikin_impedances = Gains.impedances;
Gains.ikin_dampings   = 2*sqrt(Gains.ikin_impedances); 
    