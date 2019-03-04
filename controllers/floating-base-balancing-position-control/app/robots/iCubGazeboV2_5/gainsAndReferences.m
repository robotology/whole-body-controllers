% GAINSANDREFERENCES compute gains matrices, references and regularization
%                    parameters for the controller for a specific robot

%% --- Initialization ---

% references for the CoM (sine trajectory)
Config.referencesCoM.amplitude = 0.02; %[m]
Config.referencesCoM.frequency = 0.25; %[Hz]
Config.referencesCoM.direction = [0;1;0];

% pseudoinverse tolerances
Config.Reg.pinv_tol        = 1e-4;
Config.Reg.pinvDampBaseVel = 1e-4;

% Gains matrices

% CoM gains
Config.Gain.KP_CoM    = diag([50;50;50]);
Config.Gain.KD_CoM    = 2*sqrt(Config.Gain.KP_CoM);

% Feet pose Gains
Config.Gain.KP_feet   = diag([50;50;50]);
Config.Gain.KD_feet   = 2*sqrt(Config.Gain.KP_feet);

% Postural task gains         % torso    % lArm       % rArm       % lLeg             % rLeg
Config.Gain.KP_joints = diag([20;20;20; 10;10;10;10; 10;10;10;10; 30;30;20;30;40;40; 30;30;20;30;40;40]);
Config.Gain.KD_joints = 2*sqrt(Config.Gain.KP_joints);