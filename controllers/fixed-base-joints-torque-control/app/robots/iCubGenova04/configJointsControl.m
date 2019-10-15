% CONFIGJOINTSCONTROL configures all the options associated to the
%                     joints controller.
%

%% --- Initialization ---

% Default behaviour: gravity compensation. If Config.MOVE_JOINTS = true,
% the robot will also move all actuated joints following a sine trajectory
Config.MOVE_JOINTS = false;

% Max unsigned difference between two consecutive (measured) joint positions, 
% i.e. delta_jointPos = abs(jointPos(k) - jointPos(k-1)) [rad]
Sat.maxJointsPositionDelta = 15*pi/180;