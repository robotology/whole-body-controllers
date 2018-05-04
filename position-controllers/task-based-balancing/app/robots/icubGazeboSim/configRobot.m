% CONFIGROBOT initializes parameters specific of a particular robot
%             (e.g., icuGazeboSim)
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

% Syncronization with Gazebo simulator
Config.ON_GAZEBO                     = true;

% If true, the IMU orientation is used in order to estimate the
% base-to-world transformation matrix
Config.USE_IMU4EST_BASE              = false;

% If true, the orientation provided by the IMU is corrected using the neck
% positions (requires the neck port to be active)
Config.CORRECT_IMU_WITH_NECK_POS     = false;

% If true, IMU pitch and yaw are not considered for estimating base-to-world transform
Config.FILTER_IMU_YAW                = false;
Config.FILTER_IMU_PITCH              = false;

% Frames name list
Frames.IMU                           = 'imu_frame';
Frames.COM                           = 'com';
Frames.BASE_LINK                     = 'root_link';
Frames.LEFT_FOOT                     = 'l_sole';
Frames.RIGHT_FOOT                    = 'r_sole';
Frames.ROT_TASK_LINK                 = 'neck_2';

% Dimension of the joint space
ROBOT_DOF                            = 23;

% Robot configuration for WBT3.0
WBTConfigRobot                       = WBToolbox.Configuration;
WBTConfigRobot.RobotName             = 'icubSim';
WBTConfigRobot.UrdfFile              = 'model.urdf';
WBTConfigRobot.LocalName             = 'WBT';

WBTConfigRobot.ControlBoardsNames    = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints      = {'torso_pitch','torso_roll','torso_yaw', ...
                                        'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                                        'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                                        'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                                        'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

% Ports name list (requires RobotName to be setted)
Ports.LEFT_FOOT_EXT_WRENCH           = '/wholeBodyDynamics/left_foot/cartesianEndEffectorWrench:o';
Ports.RIGHT_FOOT_EXT_WRENCH          = '/wholeBodyDynamics/right_foot/cartesianEndEffectorWrench:o';
Ports.IMU                            = ['/' WBTConfigRobot.RobotName '/inertial'];
Ports.NECK_POS                       = ['/' WBTConfigRobot.RobotName '/head/state:o'];
