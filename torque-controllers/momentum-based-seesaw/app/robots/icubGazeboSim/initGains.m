ROBOT_DOF = 23;

% Robot configuration for WBT3.0
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icubSim';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

WBTConfigRobot.ControlBoardsNames = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints   = {'torso_pitch','torso_roll','torso_yaw', ...
                                     'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                                     'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                                     'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                                     'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
CONFIG.ON_GAZEBO = true;

% Update ports according to the simulation
PORTS.IMU = '/icubSim/inertial';
PORTS.IMU_SEESAW = '/seesaw/inertial';
PORTS.NECK = '/icubSim/head/state:o';
sat.torque = 40;

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

%% Seesaw parameters
seesaw = struct;

% Height of the seesaw
%  ___________
% |           |  ---> vertical border
%  *         *   | 
%   *      *     | hArc
%     *  *       |
seesaw.hArc = 0.1;
seesaw.metalPlate = 0.004;
seesaw.verticalBorder = 0.0;
seesaw.h = seesaw.hArc + seesaw.verticalBorder;

% Radius of the seesaw
seesaw.rho = 0.362;

% Height of seesaw CoM
shiftCoM = -seesaw.metalPlate*0.5;
seesaw.CoM_z =  seesaw.h - shiftCoM;

% Seesaw inertia and mass (no IMU)
seesaw.mass = 4.2;

% INERTIA TENSOR:
% Ixx Ixy Ixz 7.6698599e-02 0.0000000e+00 0.0000000e+00
% Iyx Iyy Iyz 0.0000000e+00 3.7876787e-02 0.0000000e+00
% Izx Izy Izz 0.0000000e+00 0.0000000e+00 1.0893139e-01
seesaw.inertia = diag([7.6698599e-02, 3.7876787e-02, 1.0893139e-01]);

% Distance between the center of rotation and the center of mass
seesaw.delta = seesaw.rho - seesaw.CoM_z;

% Distance (vertical) between the seesaw/feet contact plane and the seesaw CoM
seesaw.top = (seesaw.h + seesaw.metalPlate) - seesaw.CoM_z;

% Distance (XY) of the feet from the center of the seesaw (or seesaw xy CoM)
seesaw.lFootDistance_y =  0.11;
seesaw.rFootDistance_y = -0.11;
seesaw.lFootDistance_x = -0.01;
seesaw.rFootDistance_x = -0.01;

% Distance between the seesaw CoM and the feet
seesaw.s_sl = [seesaw.lFootDistance_x;seesaw.lFootDistance_y; seesaw.top];
seesaw.s_sr = [seesaw.rFootDistance_x;seesaw.rFootDistance_y; seesaw.top];

if strcmp(FRAMES.fixedLink,'l_sole')
    
    % Link fixed with the seesaw
    seesaw.s_sFixed = seesaw.s_sl;
    
elseif strcmp(FRAMES.fixedLink,'r_sole')
    
    seesaw.s_sFixed = seesaw.s_sr;
    
else
    error('The robot frame which is assumed to be fixed w.r.t. the seesaw is not valid!')    
end

% Adjust seesaw angle measurements (roll, pitch, yaw) [deg]
seesaw.offset = [0; 0; 0];

% Relative rotation between world frame and IMU seesaw world frame
seesaw.w_R_wImu = eye(3); 

%% Filter seesaw measurements

% Both velocity and orientation filters compute the average of a
% given number of samples coming from the IMU seesaw measurements. The
% number of samples is the order of the filter.

% Select the order of the filter for seesaw orientation and ang velocity
seesaw.positionFilterOrder = 2;
seesaw.velocityFilterOrder = 2;

%% References for CoM trajectory
directionOfOscillation = [0; 1; 0];
referenceParams = [0.0, 0.25]; % referenceParams(1) = amplitude of ascillations in meters; referenceParams(2) = frequency of ascillations in Hertz
noOscillationTime = 0; % the variable noOscillationTime is the time, in seconds, that the robot waits before starting moving the CoM left-and-right

%% Gains and regularization terms (for all different controllers)
if CONFIG.CONTROL_TYPE == 1    

    gain.impedances = diag([10 20 20,   10 10 10 8,   10 10 10 8,   60 60 60 60 10 10,   60 60 60 60 10 10]);                    
    gain.dampings = 2*sqrt(gain.impedances)/10;

    gain.PAngularMomentum = diag([10 15 10]);
    gain.DAngularMomentum = 2*sqrt(gain.PAngularMomentum);
    
    gain.PCOM = diag([20 10 20]);
    gain.DCOM = 2*sqrt(gain.PCOM)/20;

    % Saturate the CoM position error
    gain.P_SATURATION = 0.30;
 
    % Regularization terms
    reg = struct;
    reg.pinvDamp = 0.1;
    reg.HessianQP = 1e-7;
    reg.pinvTol = 1e-7;
    reg.pinvTolVb = 1e-4;    

elseif CONFIG.CONTROL_TYPE == 2
    
    gain.impedances = diag([10 10 20,   10 10 10 8,   10 10 10 8,   60 60 60 60 10 10,   60 60 60 60 10 10]);                    
    gain.dampings = 2*sqrt(gain.impedances)/10;

    gain.PAngularMomentum = diag([10 10 10]);
    gain.DAngularMomentum = 2*sqrt(gain.PAngularMomentum);
    
    gain.PCOM = diag([20 5 20]);
    gain.DCOM = 2*sqrt(gain.PCOM)/10;

    % Saturate the CoM position error
    gain.P_SATURATION = 0.30;
 
    % Regularization terms
    reg = struct;
    reg.pinvDamp = 0.1;
    reg.HessianQP = 1e-7;
    reg.pinvTol = 1e-7;
    reg.pinvTolVb = 1e-4;
    
end   

%% Friction cone parameters
numberOfPoints = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                    % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 
% Friction parameters
forceFrictionCoefficient = 1;   
torsionalFrictionCoefficient = 1/75;

% Physical size of foot 
gain.footSize = [-0.07  0.12 ;    % xMin, xMax
                 -0.045 0.05 ];   % yMin, yMax     
% Minimal normal force
fZmin = 20;
