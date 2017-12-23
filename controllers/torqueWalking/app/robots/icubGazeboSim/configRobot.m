% CONFIGROBOT collections of parameters specific of a particular robot
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

% Dimension of the joint space
ROBOT_DOF = 23;

% Joint list and robot name for configuring WBToolbox
WBT_wbiList   = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';
WBT_robotName = 'icubSim';

%% Constraints for QP for balancing - friction cone - z-moment - in terms of f

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the 
% circle in each cicle's quadrant 
numberOfPoints               = 4; 
forceFrictionCoefficient     = 1;  
torsionalFrictionCoefficient = 1/75;

% Min vertical force
fZmin                        = 10; % [N]

% Size of the foot
Config.footSize              = [-0.05  0.10;     % xMin, xMax
                                -0.025 0.025];   % yMin, yMax  
                            