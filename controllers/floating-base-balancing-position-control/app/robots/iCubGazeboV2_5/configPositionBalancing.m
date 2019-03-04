% CONFIGPOSITIONBALANCING configuration parameters related to the controller
%                         for the specific robot

%% --- Initialization ---

% booleans that describe the feet contacts activation/deactivation.
% If set to 1, the associated foot is in contact. Format: [l_sole; r_sole]
Config.feetContactStatus  = [1, 1];

% if TRUE, the controller will STOP if the joints hit the joints limits
% and/or if the (unsigned) difference between two consecutive joints
% encoders measurements is greater than a given threshold.
Config.EMERGENCY_STOP_WITH_JOINTS_LIMITS  = false;
Config.EMERGENCY_STOP_WITH_ENCODER_SPIKES = true;

% If TRUE, the CoM references are updated in order to follow a sinusoidal
% trajectory.
Config.COM_SINE_MOVEMENTS = true;

% Max unsigned difference between two consecutive (measured) joint positions, 
% i.e. delta_jointPos = abs(jointPos(k) - jointPos(k-1)) [rad]
Sat.maxJointsPositionDelta = 15*pi/180;