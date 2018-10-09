% INITSTATEMACHINESTANDUP initializes the robot configuration for running
%                         'STANDUP' demo. 
% 

%% --- Initialization ---

% Feet in contact (COORDINATOR DEMO ONLY)
Config.LEFT_RIGHT_FOOT_IN_CONTACT = [1 1];

% Initial foot on ground. If false, right foot is used as default contact
% frame (this does not means that the other foot cannot be in contact too).
% (COORDINATOR DEMO ONLY)
Config.LEFT_FOOT_IN_CONTACT_AT_0 = true;

% If true, the robot CoM will follow a desired reference trajectory (COORDINATOR DEMO ONLY)
Config.DEMO_MOVEMENTS = false; 

% If equal to one, the desired streamed values of the postural tasks are
% smoothed internally 
Config.SMOOTH_JOINT_DES = true;   

% torque saturation
Sat.torque = 60;

%% Regularization parameters
Reg.pinvDamp_nu_b  = 1e-3;
Reg.pinvDamp       = 1; 
Reg.pinvTol        = 1e-5;
Reg.impedances     = 0.1;
Reg.dampings       = 0;
Reg.HessianQP      = 1e-2; 
Reg.norm_tolerance = 1e-4;
                            
%% COM AND JOINT GAINS 
Gain.KP_COM     =      [250   250  50;     % state ==  1  BALANCING ON THE LEGS
                        250   250  50;     % state ==  2  MOVE COM FORWARD
                        250   250  50;     % state ==  3  TWO FEET BALANCING
                        250   250  50];    % state ==  4  LIFTING UP

Gain.KD_COM = 2*sqrt(Gain.KP_COM)/10;

Gain.KP_AngularMomentum  = 1;
Gain.KD_AngularMomentum  = 2*sqrt(Gain.KP_AngularMomentum);

%                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%        LEFT LEG            %%         RIGHT LEG          %% 
Gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  1  BALANCING ON THE LEGS
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  2  MOVE COM FORWARD
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  3  TWO FEET BALANCING
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50];  % state ==  4  LIFTING UP

Gain.impedances(3,:) = Gain.impedances(3,:)./2;      
Gain.dampings        = 0*sqrt(Gain.impedances(1,:));  

% Smoothing time gain scheduling (STANDUP DEMO ONLY)
Gain.SmoothingTimeGainScheduling = 2;

%% STATE MACHINE PARMETERS

% smoothing time for joints and CoM
Sm.smoothingTimeCoM_Joints       = [1;    % state ==  1  BALANCING ON THE LEGS
                                    2.5;  % state ==  2  MOVE COM FORWARD
                                    2;    % state ==  3  TWO FEET BALANCING
                                    4];   % state ==  4  LIFTING UP 
  
% if Sm.smoothingTimeCoM_Joints = 0, this will allow to smooth anyway the
% CoM reference trajectory                                
Sm.smoothingTimeCoM = 0.5;

% contact forces threshold (YOGA DEMO ONLY)
Sm.wrench_thresholdContactLFoot  = [0;    % NOT USED
                                    75;   % state ==  2  MOVE COM FORWARD
                                    95;  % state ==  3  TWO FEET BALANCING
                                    0];   % NOT USED

Sm.wrench_thresholdContactRFoot  = [0     % NOT USED
                                    75;   % state ==  2  MOVE COM FORWARD
                                    95;  % state ==  3  TWO FEET BALANCING
                                    0];   % NOT USED
                     
% external forces at arms threshold                    
Sm.wrench_thresholdContactRHand  = [10    % state ==  1  BALANCING ON THE LEGS
                                    0;    % NOT USED
                                    0;    % NOT USED
                                    0];   % NOT USED

                      
Sm.wrench_thresholdContactLHand  = [10    % state ==  1  BALANCING ON THE LEGS
                                    0;    % NOT USED
                                    0;    % NOT USED
                                    0];   % NOT USED

% initial state for state machine (STANDUP DEMO ONLY)
Sm.stateAt0 = 1;

% delta to be summed to the reference CoM position (STANDUP DEMO ONLY)

Sm.CoM_delta        = [% THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT LEG
                       0.0     0.0     0.0;       % NOT USED
                       0.1277 -0.0295  0.0;       % state ==  2  MOVE COM FORWARD
                       0.04   -0.0     0.0;       % state ==  3  TWO FEET BALANCING
                       0.0     0.0     0.20];     % state ==  4  LIFTING UP

% configuration parameters for state machine (STANDUP DEMO ONLY) 
Sm.tBalancing           = 3;

%% Joint references (STANDUP DEMO ONLY)

                                  %Hip pitch  %Hip roll  %Knee     %Ankle pitch  %Shoulder pitch  %Shoulder roll  %Shoulder yaw   %Elbow   %Torso pitch                        
Sm.joints_standUpPositions     = [0.0000      0.0000     0.0000    0.0000        0.0000           0.4363          0.0000          0.0000   0.0000;   % state ==  1  THIS REFERENCE IS NOT USED
                                  1.5402      0.1594    -1.7365   -0.2814       -1.6455           0.4363          0.5862          0.2473   0.4363;   % state ==  2  MOVE COM FORWARD
                                  1.1097      0.0122    -0.8365   -0.0714       -1.4615           0.4363          0.1545          0.2018   0.0611;   % state ==  3  TWO FEET BALANCING
                                  0.2094      0.1047    -0.1745   -0.0349       -1.6455           0.4363          0.5862          0.2473   0.0000];  % state ==  4  LIFTING UP

%% References for CoM trajectory (COORDINATOR DEMO ONLY)

% that the robot waits before starting the left-and-right 
Config.noOscillationTime       = 0;   
Config.directionOfOscillation  = [0;0;0];
Config.amplitudeOfOscillation  = 0.0;  
Config.frequencyOfOscillation  = 0.0;

%% Parameters for motors reflected inertia

% transmission ratio
Config.Gamma = 0.01*eye(ROBOT_DOF);

% modify the value of the transmission ratio for the hip pitch. 
% TODO: avoid to hard-code the joint numbering
Config.Gamma(end-5, end-5)  = 0.0067;
Config.Gamma(end-11,end-11) = 0.0067;

% motors inertia (Kg*m^2)
legsMotors_I_m           = 0.0827*1e-4;
torsoPitchRollMotors_I_m = 0.0827*1e-4;
torsoYawMotors_I_m       = 0.0585*1e-4;
armsMotors_I_m           = 0.0585*1e-4;
Config.I_m               = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                 torsoYawMotors_I_m;
                                 armsMotors_I_m*ones(8,1);
                                 legsMotors_I_m*ones(12,1)]);

% parameters for coupling matrices. Updated according to the wiki:
%
% http://wiki.icub.org/wiki/ICub_coupled_joints 
%
% and corrected according to https://github.com/robotology/robots-configuration/issues/39
t  = 0.615;
r  = 0.022;
R  = 0.04;

% coupling matrices
T_LShoulder = [-1  0  0;
               -1 -t  0;
                0  t -t];

T_RShoulder = [ 1  0  0;
                1  t  0;
                0 -t  t];

T_torso = [ 0.5    -0.5     0;
            0.5     0.5     0;
            r/(2*R) r/(2*R) r/R];
       
if Config.INCLUDE_COUPLING
       
    Config.T = blkdiag(T_torso,T_LShoulder,1,T_RShoulder,1,eye(12));
else          
    Config.T = eye(ROBOT_DOF);
end

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff  = 0;

%% Constraints for QP for balancing

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the circle 
% in each cicle's quadrant
numberOfPoints               = 4;  
forceFrictionCoefficient     = 1/3;    
torsionalFrictionCoefficient = 1/75;
fZmin                        = 10;

% physical size of the foot                             
feet_size                    = [-0.05  0.10;     % xMin, xMax
                                -0.025 0.025];   % yMin, yMax 
                            
leg_size                     = [-0.025  0.05 ;  % xMin, xMax
                                -0.025  0.025]*2; % yMin, yMax 
                            
