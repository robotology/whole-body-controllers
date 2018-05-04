% Gains and parameters for impedance controller
Config.ON_GAZEBO = false;
ROBOT_DOF        = 23;

% Robot configuration for WBT3.0
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icub';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

WBTConfigRobot.ControlBoardsNames = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints   = {'torso_pitch','torso_roll','torso_yaw', ...
                                     'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                                     'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                                     'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                                     'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
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

%% Parameters for including friction

% transmission ratio
Config.Gamma = 0.01*eye(ROBOT_DOF);

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
       
Config.T = blkdiag(T_torso,T_LShoulder,1,T_RShoulder,1,eye(12));

% Multiplier of viscous friction as seen in the motors dynamics
Kbemf = [0.00125  0.00125  0.00125 ...                    % torso
         0.002  0.0005 0.0007 0.0007 ...                  % left arm
         0.002  0.0005 0.0007 0.0007 ...                  % right arm
         0.0035 0.002  0.002  0.002  0.0025  0.0025 ...   % left leg
         0.004  0.002  0.002  0.002  0.0025  0.0025];     % right leg
     
% Multiplier of viscous friction as seen in the joint dynamics
invTGamma_transpose = eye(size(Config.Gamma))/(transpose(Config.T*Config.Gamma));
invTGamma           = eye(size(Config.Gamma))/(Config.T*Config.Gamma);

if EXPLOIT_FRICTION

    Gain.Kf = invTGamma_transpose*diag(Kbemf)*invTGamma;
else
    Gain.Kf = zeros(23);
end
