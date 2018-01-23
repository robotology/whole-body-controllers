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
Config.ON_GAZEBO = true;

% Config.USE_MOTOR_REFLECTED_INERTIA: if set to true, motors reflected
% inertias are included in the system mass matrix. If
% Config.INCLUDE_COUPLING is true, then the coupling effect (some joints
% motion is the result of more than one motor motion) is taken into account.
Config.USE_MOTOR_REFLECTED_INERTIA = false;
Config.INCLUDE_COUPLING            = false;

% Dimension of the joint space
ROBOT_DOF = 23;

% Robot configuration for WBT3.0
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icubSim';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

WBTConfigRobot.ControlBoardsNames = {'right_arm','left_arm','right_leg','left_leg','torso'};
WBTConfigRobot.ControlledJoints   = {'torso_pitch','torso_roll','torso_yaw', ...
                                     'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                                     'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                                     'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                                     'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

% Ports name list (requires RobotName to be setted)
Ports.LEFT_FOOT_EXT_WRENCH  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
Ports.RIGHT_FOOT_EXT_WRENCH = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';
Ports.IMU                   = ['/' WBTConfigRobot.RobotName '/inertial'];
Ports.NECK_POS              = ['/' WBTConfigRobot.RobotName '/head/state:o'];

% Ports for connecting the model with the MPC controller
PORTS.COM_DES = '/walking-coordinator/com:o';
PORTS.Q_DES = '/walking-coordinator/joints:o';
PORTS.LFOOT_DES = '/walking-coordinator/leftFoot:o';
PORTS.RFOOT_DES = '/walking-coordinator/rightFoot:o';
PORTS.ACTIVE_CONTACTS = '/walking-coordinator/contact:o';
PORTS.LFOOT_IS_FIXED = '/walking-coordinator/leftStanding:o';
PORTS.ACK = '/walking-coordinator/done:i';
                            