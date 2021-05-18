%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON ROBOT CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Init simulator core physics paramaters
physics_config.GRAVITY_ACC = [0,0,-9.81];
physics_config.TIME_STEP = Config.tStepSim;

% Robot configuration for WBToolbox
WBTConfigRobotSim = WBToolbox.Configuration;
WBTConfigRobotSim.RobotName = 'icubSim';
WBTConfigRobotSim.UrdfFile = 'model.urdf';
WBTConfigRobotSim.LocalName = 'WBT';
WBTConfigRobotSim.GravityVector = physics_config.GRAVITY_ACC;

% Controlboards and joints list. Each joint is associated to the corresponding controlboard
WBTConfigRobotSim.ControlBoardsNames = {'torso','left_arm','right_arm','left_leg','right_leg'}; %,'head'};
WBTConfigRobotSim.ControlledJoints = [];
numOfJointsForEachControlboard = [];

ControlBoards = struct();
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{1}) = {'torso_pitch','torso_roll','torso_yaw'};
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{2}) = {'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow'};
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{3}) = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow'};
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{4}) = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{5}) = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
% ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{6}) = {'neck_pitch','neck_roll','neck_yaw'};

for n = 1:length(WBTConfigRobotSim.ControlBoardsNames)
    WBTConfigRobotSim.ControlledJoints = [WBTConfigRobotSim.ControlledJoints, ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{n})];
    numOfJointsForEachControlboard = [numOfJointsForEachControlboard; length(ControlBoards.(WBTConfigRobotSim.ControlBoardsNames{n}))];
end

% structure used to configure the Robot class
% 
robot_config.jointOrder = WBTConfigRobotSim.ControlledJoints;
robot_config.numOfJointsForEachControlboard = numOfJointsForEachControlboard;

% Note: Since iDynTree 3.0.0, if meshFilePrefix='', the standard iDynTree workflow of locating the
% mesh via the ExternalMesh.getFileLocationOnLocalFileSystem method is used. The iCub model meshes
% file tree is compatible with this workflow.
robot_config.meshFilePrefix = '';
robot_config.fileName = WBTConfigRobotSim.UrdfFile;
robot_config.N_DOF = numel(WBTConfigRobotSim.ControlledJoints);
robot_config.N_DOF_MATRIX = eye(robot_config.N_DOF);

% Initial condition of iCub and for the integrators.
initialConditions.base_position = [0; 0; 0.619];
initialConditions.orientation = diag([-1, -1, 1]);
initialConditions.w_H_b = mwbs.Utils.Rp2H(initialConditions.orientation, initialConditions.base_position);

% joint (inital) position
initialConditions.s = [
    0; 0; 0; ...
    -35.97; 29.97; 0.06; 50.00; ...
    -35.97; 29.97; 0.06; 50.00; ...
    10; 0; 0; -20; -10; 0; ...
    10; 0; 0; -20; -10; 0]*pi/180;

% velocty initial conditions
initialConditions.base_linear_velocity = [0; 0; 0];
initialConditions.base_angular_velocity = [0; 0; 0];
initialConditions.base_pose_dot = [initialConditions.base_linear_velocity; initialConditions.base_angular_velocity];
initialConditions.s_dot = zeros(robot_config.N_DOF, 1);

robot_config.initialConditions = initialConditions;

% Reflected inertia
robot_config.SIMULATE_MOTOR_REFLECTED_INERTIA = true;
INCLUDE_COUPLING = true;

% Robot frames list
FramesSim.BASE = 'root_link';
FramesSim.IMU = 'imu_frame';
FramesSim.LEFT_FOOT = 'l_sole';
FramesSim.RIGHT_FOOT = 'r_sole';
FramesSim.COM = 'com';

robot_config.robotFrames = FramesSim;

% structure used to configure the Contacts class
% 

% foot print of the feet (iCub)
vertex = zeros(3, 4);
vertex(:, 1) = [-0.06; 0.04; 0];
vertex(:, 2) = [0.11; 0.04; 0];
vertex(:, 3) = [0.11; -0.035; 0];
vertex(:, 4) = [-0.06; -0.035; 0];

contact_config.foot_print = vertex;
contact_config.total_num_vertices = size(vertex,2)*2;

% friction coefficient for the feet
contact_config.friction_coefficient = 0.1;

%% Motors reflected inertia

% transmission ratio (1/N)
Gamma                 = 0.01*eye(ROBOT_DOF);

% modify the value of the transmission ratio for the hip pitch. 
% TODO: avoid to hard-code the joint numbering
Gamma(end-5, end-5)   = 0.0067;
Gamma(end-11,end-11)  = 0.0067;

% motors inertia (Kg*m^2)
legsMotors_I_m               = 0.0827*1e-4;
torsoPitchRollMotors_I_m     = 0.0827*1e-4;
torsoYawMotors_I_m           = 0.0585*1e-4;
armsMotors_I_m               = 0.0585*1e-4;

% add harmonic drives reflected inertia
legsMotors_I_m           = legsMotors_I_m + 0.054*1e-4;
torsoPitchRollMotors_I_m = torsoPitchRollMotors_I_m + 0.054*1e-4;
torsoYawMotors_I_m       = torsoYawMotors_I_m + 0.021*1e-4;
armsMotors_I_m           = armsMotors_I_m + 0.021*1e-4;
 
I_m                   = diag([torsoPitchRollMotors_I_m*ones(2,1);
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
       
if INCLUDE_COUPLING
    T = blkdiag(T_torso,T_LShoulder,1,T_RShoulder,1,eye(12));
else          
    T = eye(robot_config.N_DOF);
end

motorsReflectedInertia = wbc.computeMotorsReflectedInertia(Gamma,T,I_m);

% Joint friction

% === Mapping ===
jointDefaultOrder = {...
    'torso_pitch','torso_roll','torso_yaw', ...
    'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
    'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
    'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
    'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

KvmechMappingTorso = 0.2*ones(3,1);
KvmechMappingLeftArm = 0.2*ones(4,1);
KvmechMappingRightArm = 0.2*ones(4,1);
KvmechMappingLeftLeg = [0.2 0.2 0.2 0.2 0.6 0.6]';
KvmechMappingRightLeg = [0.2 0.2 0.2 0.2 0.6 0.6]';

KvmechMapping = containers.Map(...
    jointDefaultOrder, ...
    [
    KvmechMappingTorso
    KvmechMappingLeftArm
    KvmechMappingRightArm
    KvmechMappingLeftLeg
    KvmechMappingRightLeg
    ]);

KvmechMat = diag(cell2mat(KvmechMapping.values(robot_config.jointOrder)));

jointFrictionMat = wbc.computeMotorsReflectedInertia(eye(robot_config.N_DOF),T,KvmechMat);

%% size of the square you see around the robot
visualizerAroundRobot = 1; % mt

clear ControlBoards numOfJointsForEachControlboard FramesSim initialConditions vertex
