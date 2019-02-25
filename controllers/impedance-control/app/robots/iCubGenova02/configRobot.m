% CONFIGROBOT initializes parameters specific of a particular robot
%             (e.g., icuGazeboSim)
%

%% --- Initialization ---

% Gains and parameters for impedance controller
Config.ON_GAZEBO = false;
ROBOT_DOF        = 23;

% max unsigned difference between two consecutive (measured) joint positions, 
% i.e. delta_qj = abs(qj(k) - qj(k-1))
Sat.maxJointsPositionDelta = 15*pi/180; % [rad] 

% Robot configuration for WBToolbox
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icub';
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

% References for the demo movements
if MOVING
    
    % Impedance gains
    Kp     = 20*diag(ones(1,ROBOT_DOF));
    Kd     = 2*sqrt(Kp)*0;
    
    AMPLS  = 7.5*ones(1,ROBOT_DOF);
    FREQS  = 0.5*ones(1,ROBOT_DOF);
   
else
    
    % Impedance gains
    Kp     = 0*diag(ones(1,ROBOT_DOF));
    Kd     = 0*diag(ones(1,ROBOT_DOF));
    
    AMPLS  = 0*ones(1,ROBOT_DOF);
    FREQS  = 0*ones(1,ROBOT_DOF);
end
    
if size(Kp,1) ~= ROBOT_DOF
    error('Dimension of Kp different from ROBOT_DOF')
end