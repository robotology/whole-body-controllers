% CONFIGROBOT initializes parameters specific of a particular robot
%             (e.g., icubGazeboSim)

%% --- Initialization ---

Config.ON_GAZEBO         = true;
ROBOT_DOF                = 23;
Config.GRAV_ACC          = 9.81;

% Robot configuration for WBToolbox
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'ergocubSim';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

% Controlboards and joints list. Each joint is associated to the corresponding controlboard 
WBTConfigRobot.ControlBoardsNames     = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints       = [];
Config.numOfJointsForEachControlboard = [];

ControlBoards                                        = struct();
ControlBoards.(WBTConfigRobot.ControlBoardsNames{1}) = {'torso_pitch','torso_roll','torso_yaw'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{2}) = {'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{3}) = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{4}) = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{5}) = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

for n = 1:length(WBTConfigRobot.ControlBoardsNames)

    WBTConfigRobot.ControlledJoints       = [WBTConfigRobot.ControlledJoints, ...
                                             ControlBoards.(WBTConfigRobot.ControlBoardsNames{n})];
    Config.numOfJointsForEachControlboard = [Config.numOfJointsForEachControlboard; length(ControlBoards.(WBTConfigRobot.ControlBoardsNames{n}))];
end

% Frames list
Frames.BASE       = 'root_link'; 
Frames.IMU        = 'head_imu_0';
Frames.LEFT_FOOT  = 'l_sole';
Frames.RIGHT_FOOT = 'r_sole';
Frames.COM        = 'com';

% Config.SATURATE_TORQUE_DERIVATIVE: if true, the derivative of the control
% input is saturated. In this way, it is possible to reduce high frequency
% oscillations and discontinuities in the control input.
Config.SATURATE_TORQUE_DERIVATIVE         = false;

% if TRUE, the controller will STOP if the joints hit the joints limits
% and/or if the (unsigned) difference between two consecutive joints
% encoders measurements is greater than a given threshold.
Config.EMERGENCY_STOP_WITH_JOINTS_LIMITS  = false;
Config.EMERGENCY_STOP_WITH_ENCODER_SPIKES = true;

% Config.USE_MOTOR_REFLECTED_INERTIA: if set to true, motors reflected
% inertias are included in the system mass matrix. If
% Config.INCLUDE_COUPLING is true, then the coupling effect (some joints
% motion is the result of more than one motor motion) is taken into account.
% Config.INCLUDE_HARMONIC_DRIVE_INERTIA is true, then the harmonic drive
% reflected inertia is also considered
Config.USE_MOTOR_REFLECTED_INERTIA    = false;
Config.INCLUDE_COUPLING               = false;
Config.INCLUDE_HARMONIC_DRIVE_INERTIA = false;

% Config.USE_IMU4EST_BASE: if set to false, the base frame is estimated by 
% assuming that either the left or the right foot stay stuck on the ground. 
% Which foot the controller uses depends on the contact forces acting on it. 
% If set to true, the base orientation is estimated by using the IMU, while
% the base position by assuming that the origin of either the right or the
% left foot do not move. 
Config.USE_IMU4EST_BASE = false;

% Config.YAW_IMU_FILTER when the flag Config.USE_IMU4EST_BASE = true, then 
% the orientation of the floating base is estimated as explained above. However,
% the foot is usually perpendicular to gravity when the robot stands on flat 
% surfaces, and rotation about the gravity axis may be affected by the IMU drift 
% in estimating this angle. Hence, when either of the flags Config.YAW_IMU_FILTER
% is set to true, then the yaw angle of the contact foot is ignored and kept 
% equal to the initial value.
Config.FILTER_IMU_YAW   = true;

% Config.CORRECT_NECK_IMU: when set equal to true, the kinematics from the
% IMU and the contact foot is corrected by using the neck angles. If it set
% equal to false, recall that the neck is assumed to be in (0,0,0). Valid
% ONLY while using the ICUB HEAD IMU!
Config.CORRECT_NECK_IMU = false;

% Config.USE_QP_SOLVER: if set to true, a QP solver is used to account for 
% inequality constraints of contact wrenches.
Config.USE_QP_SOLVER    = true; 

% Ports name list
Ports.WRENCH_LEFT_FOOT  = '/wholeBodyDynamics/left_foot_rear/cartesianEndEffectorWrench:o';
Ports.WRENCH_RIGHT_FOOT = '/wholeBodyDynamics/right_foot_rear/cartesianEndEffectorWrench:o';
Ports.IMU               = ['/' WBTConfigRobot.RobotName '/inertial'];
Ports.NECK_POS          = ['/' WBTConfigRobot.RobotName '/head/state:o'];

% Ports dimensions
Ports.NECK_POS_PORT_SIZE         = 3;
Ports.IMU_PORT_SIZE              = 12;
Ports.IMU_PORT_ORIENTATION_INDEX = [1,2,3];
Ports.WRENCH_PORT_SIZE           = 6;