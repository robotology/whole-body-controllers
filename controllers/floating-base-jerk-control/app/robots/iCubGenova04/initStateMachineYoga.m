% INITSTATEMACHINEYOGA initializes the robot configuration for running
%                      'YOGA' demo. 
% 

%% --- Initialization ---

% Feet in contact (COORDINATOR DEMO ONLY)
Config.LEFT_RIGHT_FOOT_IN_CONTACT = [1 1];

% Initial foot on ground. If false, right foot is used as default contact
% frame (this does not means that the other foot cannot be in contact too).
% (COORDINATOR DEMO ONLY)
Config.LEFT_FOOT_IN_CONTACT_AT_0  = true;

% If true, the robot CoM will follow a desired reference trajectory (COORDINATOR DEMO ONLY)
Config.DEMO_MOVEMENTS             = false;

% If equal to one, the desired streamed values of the center of mass 
% are smoothed internally 
Config.SMOOTH_COM_DES             = true;   

% If equal to one, the desired streamed values of the postural tasks are
% smoothed internally  
Config.SMOOTH_JOINT_DES           = true;   
 
% torque saturation
Sat.torque                        = 60;

% parametrization derivative saturation
Sat.xiDot                         = 10;

% threshold for the deadzone
Sat.deadZoneThreshold             = 0.0001;

% torque derivative max absolute value
Sat.uDotMax                       = 300;

% max unsigned difference between two consecutive (measured) joint positions, 
% i.e. delta_qj = abs(qj(k) - qj(k-1))
Sat.maxJointsPositionDelta        = 15*pi/180; % [rad] 

%% Regularization parameters
Reg.pinvDamp_baseVel = 1e-7;
Reg.pinvDamp         = 0.07; 
Reg.pinvTol          = 1e-5; 
                            
%% COM AND POSTURAL GAINS 

% parameters for both two feet and one foot balancing
Gain.KO_momentum = 1*eye(6);

Gain.KP_COM = [20    20  5  % state ==  1  TWO FEET BALANCING
               20    20  5  % state ==  2  COM TRANSITION TO LEFT 
               20    30  5  % state ==  3  LEFT FOOT BALANCING
               20    30  5  % state ==  4  YOGA LEFT FOOT 
               20    30  5  % state ==  5  PREPARING FOR SWITCHING 
               30    30  5  % state ==  6  LOOKING FOR CONTACT
               20    20  5  % state ==  7  TRANSITION TO INITIAL POSITION 
               20    20  5  % state ==  8  COM TRANSITION TO RIGHT FOOT
               50    20  5  % state ==  9  RIGHT FOOT BALANCING
               50    20  5  % state == 10  YOGA RIGHT FOOT 
               50    20  5  % state == 11  PREPARING FOR SWITCHING 
               50    20  5  % state == 12  LOOKING FOR CONTACT
               50    20  5];% state == 13  TRANSITION TO INITIAL POSITION

Gain.KD_COM = 2*sqrt(Gain.KP_COM)/15;

Gain.KP_AngularMomentum  = 5;
Gain.KD_AngularMomentum  = 2*sqrt(Gain.KP_AngularMomentum)/5;

%                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
Gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   30   20    20    100 100, 30   50   30    60    100 100  % state ==  1  TWO FEET BALANCING
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   30   20    20    100 100, 30   50   30    60    100 100  % state ==  2  COM TRANSITION TO LEFT 
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    100 100, 30   30   20    20    100 100  % state ==  3  LEFT FOOT BALANCING
                    30   30   30, 10   10    10   10, 10   10    10   10,100  200  100   400    100 100,100   50   30    50    100 100  % state ==  4  YOGA LEFT FOOT 
                    30   30   30,  5    5    10   10, 10   10    20   10,200  250   80    20     10  10,220  550  220   200     65 300  % state ==  5  PREPARING FOR SWITCHING 
                    30   30   30, 10   10    20   10, 10   10    20   10,100  350   20   200     10 100,220  550  220   200     65 300  % state ==  6  LOOKING FOR CONTACT
                    10   30   40, 20   20    20   20, 30   30    30   20, 30   50   60    80      5   5, 30   30   30    20      5   5  % state ==  7  TRANSITION TO INITIAL POSITION 
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   60    30    100 100, 30   30   30    20    100 100  % state ==  8  COM TRANSITION TO RIGHT FOOT
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    100 100, 30   30   20    20    100 100  % state ==  9  RIGHT FOOT BALANCING
                    30   30   30, 10   10    10   10, 10   10    10   10,100   50   30    50    100 100,100  200  100   100     10  10  % state == 10  YOGA RIGHT FOOT 
                    30   30   30, 10   10    10   10, 10   10    10   10,220  550  220   200     65 300,200  250   20    20     10  10  % state == 11  PREPARING FOR SWITCHING 
                    30   30   30, 10   10    10   10, 10   10    10   10,220  550  220   200     65 300,100  350   20   200     10 100  % state == 12  LOOKING FOR CONTACT
                    30   40   30, 10   10    10   10, 10   10    10   10,220  550  220   200     65 300,100  350   20   200     10 100];% state == 13  TRANSITION TO INITIAL POSITION

Gain.impedances(4,1:3)   = Gain.impedances(4,1:3)*3;
Gain.impedances(6,18:23) = Gain.impedances(6,18:23)*2; 
Gain.impedances(10,1:3)  = Gain.impedances(10,1:3)*3;
Gain.impedances(13,1:3)  = Gain.impedances(13,1:3)*3;
 
Gain.dampings            = 0*sqrt(Gain.impedances);  

% Smoothing time gain scheduling (YOGA DEMO ONLY)
Gain.SmoothingTimeGainScheduling = 2;

% Joint torques minimization gains in the momentum jerk QP
Gain.useVelFeedbackForMinTorques = 0;
Gain.K_tau                       = 1;

% Integral correction (xiDot)
Gain.Ke                          = 1;

%% STATE MACHINE PARMETERS

% smoothing time for joints and CoM
Sm.smoothingTimeCoM_Joints       = [1;    %% state ==  1  TWO FEET BALANCING
                                    1;    %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    1;    %% state ==  3  LEFT FOOT BALANCING 
                                    0.9;  %% state ==  4  YOGA LEFT FOOT
                                    2;    %% state ==  5  PREPARING FOR SWITCHING
                                    5;    %% state ==  6  LOOKING FOR CONTACT 
                                    1;    %% state ==  7  TRANSITION INIT POSITION
                                    1;    %% state ==  8  COM TRANSITION TO RIGHT FOOT
                                    1;    %% state ==  9  RIGHT FOOT BALANCING 
                                    0.9;  %% state == 10  YOGA RIGHT FOOT
                                    2;    %% state == 11  PREPARING FOR SWITCHING
                                    2;    %% state == 12  LOOKING FOR CONTACT 
                                    5];   %% state == 13  TRANSITION INIT POSITION

% scale factor smoothing time multiplies the smoothing factor during the
% Yoga (state 4 and 10). The purpose is to reduce the time necessary for 
% the reference to converge to the next position, but without changing also
% the valuse stored in Sm.joints_leftYogaRef/Sm.joints_rightYogaRef (YOGA DEMO ONLY)
Sm.scaleFactorSmoothingTime     = 0.9;
                                
% time between two yoga positions (YOGA DEMO ONLY)
Sm.joints_pauseBetweenYogaMoves = 5;

% contact forces threshold (YOGA DEMO ONLY)
Sm.wrench_thresholdContactOn      = 50;
Sm.wrench_thresholdContactOff     = 100;
Sm.wrench_minVerticalForceOneFoot = 300;

% threshold on CoM and joints error (YOGA DEMO ONLY)
Sm.CoM_threshold                = 0.02; 
Sm.joints_thresholdNotInContact = 15;
Sm.joints_thresholdInContact    = 50;

% initial state for state machine (YOGA DEMO ONLY)
Sm.stateAt0 = 1;

% delta to be summed to the reference CoM position (YOGA DEMO ONLY)

Sm.CoM_delta       = [% THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.0,   0.00,  0.0;   %% NOT USED
                      0.0,   0.00,  0.0;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                      0.0,   0.005, 0.0;   %% state ==  3  LEFT FOOT BALANCING 
                      0.0,   0.0065, 0.06; %% state ==  4  YOGA LEFT FOOT
                      0.0,   0.005,-0.00;  %% state ==  5  PREPARING FOR SWITCHING
                      0.02, -0.08,  0.0;   %% state ==  6  LOOKING FOR CONTACT 
                      0.04,  0.00,  0.0;   %% state ==  7  BALANCING
                      % THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE RIGHT FOOT
                      0.0,  0.00,   0.0;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                      0.0, -0.005,  0.0;   %% state ==  9  RIGHT FOOT BALANCING 
                      0.0, -0.005,  0.0;   %% state == 10  YOGA RIGHT FOOT
                      0.0, -0.015,  0.0;   %% state == 11  PREPARING FOR SWITCHING
                      0.02, 0.02,   0.0;   %% state == 12  LOOKING FOR CONTACT 
                      0.0,  0.00,   0.0];  %% NOT USED

% configuration parameters for state machine (YOGA DEMO ONLY) 
Sm.tBalancing               = 1;
Sm.tBalancingBeforeYoga     = 1;
Sm.yogaExtended             = true;
Sm.skipYoga                 = false;
Sm.demoOnlyBalancing        = false;
Sm.demoStartsOnRightSupport = false; % If false, the Yoga demo is performed on the left foot first
Sm.yogaAlsoOnRightFoot      = false; % TO DO: yoga on both feet starting from right foot (not available for now)

%%%% List of possible "Yoga in loop" %%%%

% the robot will repeat the FULL DEMO (two feet balancing, yoga on left
% foot, back on two feet, yoga right foot, back on two feet). The demo is
% repeated until the user stops the Simulink model. This option is ignored
% if Sm.demoStartsOnRightSupport = true.
Sm.twoFeetYogaInLoop        = false;

% the robot will repeat the ONE FOOT yoga for the number of times the user
% specifies in the Sm.yogaCounter option. The robot WILL NOT go back to two
% feet balancing in between to consecutive yoga. WARNING: if the option 
% Sm.yogaAlsoOnRightFoot is true, then the robot will repeat first the yoga
% on left foot for the number of times the user specifies in the Sm.yogaCounter,
% and then it will repeat the yoga on the right foot for the same number of times.
% This option is ignored if Sm.repeatTwiceYogaWithDifferentSpeed = true.
Sm.oneFootYogaInLoop        = false;
Sm.yogaCounter              = 10;

% the robot will repeat the yoga moveset twice. This option works as the 
% option Sm.oneFootYogaInLoop, but the yoga is repeated only twice. However,
% it is possible to set a different yoga speed for the two yoga. 
% Uneffective if Sm.yogaExtended = false;
Sm.repeatTwiceYogaWithDifferentSpeed = false;

% smoothing time for the second time the Yoga moveset are performed
Sm.smoothingTimeSecondYogaLeft       = 0.7;
Sm.smoothingTimeSecondYogaRight      = 0.7;

%% Joint references (YOGA DEMO ONLY)
Sm.joints_references = [  zeros(1,ROBOT_DOF);                                %% THIS REFERENCE IS IGNORED 
                        [-0.0348,0.0779,0.0429, ...                          %% state == 2  COM TRANSITION TO LEFT 
                         -0.1493,0.8580,0.2437,0.8710, ...                   %
                         -0.1493,0.8580,0.2437,0.8710, ...                   %
                         -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...   %  
                          0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     %  
                        [ 0.0864,0.0258,0.0152, ...                          %% state == 3  LEFT FOOT BALANCING
                          0.1253,0.8135,0.3051,0.7928, ...                   %    
                          0.0563,0.6789,0.3340,0.6214, ...                   %
                         -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...   %  
                          0.0005,0.0793,-0.0014,-0.0051,-0.1060,-0.1151];    % 
                          zeros(1,ROBOT_DOF);                                %% THIS REFERENCE IS IGNORED 
                        [-0.0348,0.0779,-0.2029, ...                          %% state == 5  PREPARING FOR SWITCHING
                         -0.1493,0.8580,0.2437,0.8710, ...                   %
                         -0.1493,0.8580,0.2437,0.8710, ...                   %
                         -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...   %  
                          0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     % 
                        [ 0.0864,0.0258,-0.2052, ...                          %% state == 6  LOOKING FOR CONTACT
                          0.1253,0.8135,0.3051,0.7928, ...                   %
                          0.0563,0.6789,0.3340,0.6214, ...                   %
                          0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369, ...  %
                         -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];      %   
                          zeros(1,ROBOT_DOF);                                %% THIS REFERENCE IS IGNORED
                        [ 0.0864,0.0258,0.0152, ...                          %% state == 8  COM TRANSITION TO RIGHT FOOT
                          0.1253,0.8135,0.3051,0.7928, ...                   %
                          0.0563,0.6789,0.3340,0.6214, ...                   %
                          0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369, ...  %
                         -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];      % 
                        [ 0.0864,0.0258,0.0152, ...                          %% state == 9  RIGHT FOOT BALANCING
                          0.1253,0.8135,0.3051,0.7928, ...                   %    
                          0.0563,0.6789,0.3340,0.6214, ...                   %
                          0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...  %  
                         -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];      %  
                          zeros(1,ROBOT_DOF);                                %% THIS REFERENCE IS IGNORED  
                        [-0.0348,0.0779,0.0429, ...                          %% state == 11  PREPARING FOR SWITCHING
                         -0.1493,0.8580,0.2437,0.8710, ...                   %
                         -0.1493,0.8580,0.2437,0.8710, ...                   %
                          0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...  %  
                         -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];      %     
                        [ 0.0864,0.0258,0.0152, ...                          %% state == 12  LOOKING FOR CONTACT
                          0.1253,0.8135,0.3051,0.7928, ...                   %
                          0.0563,0.6789,0.3340,0.6214, ...                   %
                         -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277, ...   %
                          0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369];     %   
                          zeros(1,ROBOT_DOF)];                               %% THIS REFERENCE IS IGNORED       

% YOGA MOVESET (LEFT YOGA, EXTENDED)
q1 =        [-0.0790,0.2279, 0.4519, ...
             -1.1621,0.6663, 0.4919, 0.9947, ... 
             -1.0717,1.2904,-0.2447, 1.0948, ...
              0.2092,0.2060, 0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3484,0.4008,-0.0004,-0.3672,-0.1060,-0.0875];

q2 =        [-0.0790,0.1279, 0.4519, ...
             -1.1621,0.6663, 0.4965, 0.9947, ...
             -1.0717,1.2904,-0.2493, 1.0948, ...
              0.2092,0.2060, 0.0006,-0.1741,-0.1044,0.0700, ... 
              0.3714,0.9599, 1.3253,-1.6594,-0.1060,-0.0614];
          
q3 =        [-0.0852,-0.3273,0.0821,...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092,0.2060, 0.0006,-0.1741,-0.1044,0.0700, ...
              0.3714,0.9599, 1.3253,-1.6594, 0.5000,-0.0614];
          
q4 =        [-0.0852,-0.4273,0.0821,...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.3473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3514, 1.3107,1.3253,-0.0189, 0.5000,-0.0614];
          
q5 =        [-0.0790,-0.2273, 0.4519, ...
             -1.1621,0.6663, 0.4965, 0.9947, ...
             -1.0717,1.2904,-0.2493, 1.0948, ...
              0.2092, 0.4473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3514, 1.3107,1.3253,-0.0189, 0.5000,-0.0614];
          
q6 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3514, 1.3107,1.3253,-0.0189, 0.5000,-0.0614];
          
q7 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3514, 1.3107,1.3253, -1.6217, 0.5000,-0.0614];
          
q8 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3514, 1.3107,1.3253,-0.0189, 0.5000,-0.0614];
                   
q9 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3514, 0.0107,1.3253,-0.0189, 0.5000,-0.0614];
                    
q10 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3514, 1.3107,1.3253,-0.0189, 0.5000,-0.0614];
                   
q11 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.8514, 1.3107,1.3253,-0.0189, 0.5000,-0.0614];

q12 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.8514, 0.3107,1.3253,-0.0189, 0.5000,-0.0614];
          
q13 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.8514, 1.3107,1.3253,-0.0189, 0.5000,-0.0614];
          
q14 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.8514, 0.0107,1.3253,-0.0189, 0.5000,-0.0614];
          
q15 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              1.5514, 0.3107,1.3253,-0.0189, 0.5000,-0.0614];
          
q16 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
              0.2514, 0.0107,1.3253,-0.0189, 0.5000,-0.0614];
          
q17 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700, ...
             -0.3514, 0.3107,1.3253,-0.0189, 0.5000,-0.0614];
         
q18 =        [-0.0852,-0.4273,0.0821, ...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.0100,-0.1044, 0.0700, ...
              0.3514, 1.3107,1.3253,-0.0189, 0.5000,-0.0614];
          
Sm.joints_leftYogaRef  = [ 0,                              q1;
                           1*Sm.smoothingTimeCoM_Joints(4),q2;
                           2*Sm.smoothingTimeCoM_Joints(4),q3;
                           3*Sm.smoothingTimeCoM_Joints(4),q4;
                           4*Sm.smoothingTimeCoM_Joints(4),q5;
                           5*Sm.smoothingTimeCoM_Joints(4),q6;
                           6*Sm.smoothingTimeCoM_Joints(4),q7;
                           7*Sm.smoothingTimeCoM_Joints(4),q8;
                           8*Sm.smoothingTimeCoM_Joints(4),q9;
                           9*Sm.smoothingTimeCoM_Joints(4),q10;
                          10*Sm.smoothingTimeCoM_Joints(4),q11;
                          11*Sm.smoothingTimeCoM_Joints(4),q12;
                          12*Sm.smoothingTimeCoM_Joints(4),q13;
                          13*Sm.smoothingTimeCoM_Joints(4),q14;
                          14*Sm.smoothingTimeCoM_Joints(4),q15;
                          15*Sm.smoothingTimeCoM_Joints(4),q16;
                          16*Sm.smoothingTimeCoM_Joints(4),q17;
                          17*Sm.smoothingTimeCoM_Joints(4),q10;
                          18*Sm.smoothingTimeCoM_Joints(4),q11;
                          19*Sm.smoothingTimeCoM_Joints(4),q12;
                          20*Sm.smoothingTimeCoM_Joints(4),q13;
                          21*Sm.smoothingTimeCoM_Joints(4),q14;
                          22*Sm.smoothingTimeCoM_Joints(4),q15;
                          23*Sm.smoothingTimeCoM_Joints(4),q16;
                          24*Sm.smoothingTimeCoM_Joints(4),q17;
                          25*Sm.smoothingTimeCoM_Joints(4),q18];
                 
Sm.joints_rightYogaRef      = Sm.joints_leftYogaRef;
Sm.joints_rightYogaRef(:,1) = [0,                              ;
                               1*Sm.smoothingTimeCoM_Joints(10);
                               2*Sm.smoothingTimeCoM_Joints(10);
                               3*Sm.smoothingTimeCoM_Joints(10);
                               4*Sm.smoothingTimeCoM_Joints(10);
                               5*Sm.smoothingTimeCoM_Joints(10);
                               6*Sm.smoothingTimeCoM_Joints(10);
                               7*Sm.smoothingTimeCoM_Joints(10);
                               8*Sm.smoothingTimeCoM_Joints(10);
                               9*Sm.smoothingTimeCoM_Joints(10);
                              10*Sm.smoothingTimeCoM_Joints(10);
                              11*Sm.smoothingTimeCoM_Joints(10);
                              12*Sm.smoothingTimeCoM_Joints(10);
                              13*Sm.smoothingTimeCoM_Joints(10);
                              14*Sm.smoothingTimeCoM_Joints(10);
                              15*Sm.smoothingTimeCoM_Joints(10);
                              16*Sm.smoothingTimeCoM_Joints(10);
                              17*Sm.smoothingTimeCoM_Joints(10);
                              18*Sm.smoothingTimeCoM_Joints(10);
                              19*Sm.smoothingTimeCoM_Joints(10);
                              20*Sm.smoothingTimeCoM_Joints(10);
                              21*Sm.smoothingTimeCoM_Joints(10);
                              22*Sm.smoothingTimeCoM_Joints(10);
                              23*Sm.smoothingTimeCoM_Joints(10);
                              24*Sm.smoothingTimeCoM_Joints(10);
                              25*Sm.smoothingTimeCoM_Joints(10)];

% smoothing time vector for the second time the Yoga moveset are performed
Sm.joints_leftSecondYogaRef  = Sm.smoothingTimeSecondYogaLeft.*(0:(length(Sm.joints_rightYogaRef(:,1))-1));
Sm.joints_rightSecondYogaRef = Sm.smoothingTimeSecondYogaRight.*(0:(length(Sm.joints_rightYogaRef(:,1))-1));

% keep a high smoothing time for switching from the first to the second yoga moveset
Sm.joints_leftSecondYogaRef(1)  = Sm.smoothingTimeCoM_Joints(4);
Sm.joints_rightSecondYogaRef(1) = Sm.smoothingTimeCoM_Joints(10);

% if the demo is not "yogaExtended", stop at the 8th move
% also, Sm.repeatTwiceYogaWithDifferentSpeed must be set to "false". The 
% reason is that the first and the last Yoga moveset for the "not extended"
% one are very different, and the robot may "jump" when restarting the Yoga 
% the second time
if ~Sm.yogaExtended
   
    Sm.repeatTwiceYogaWithDifferentSpeed = false;
    Sm.joints_leftYogaRef                = Sm.joints_leftYogaRef(1:8,:);
    Sm.joints_rightYogaRef               = Sm.joints_rightYogaRef(1:8,:);
    Sm.joints_leftYogaRef(8,1)           = 15*Sm.smoothingTimeCoM_Joints(4);
    Sm.joints_rightYogaRef(8,1)          = 15*Sm.smoothingTimeCoM_Joints(10);
end

% MIRROR YOGA LEFT MOVESET FOR RIGHT YOGA					 
for i = 1:size(Sm.joints_rightYogaRef,1)	
    
	Sm.joints_rightYogaRef(i,2:4)           = [Sm.joints_rightYogaRef(i,2) -Sm.joints_rightYogaRef(i,3) -Sm.joints_rightYogaRef(i,4)];
	rightArm                                =  Sm.joints_rightYogaRef(i,end-15:end-12);
	Sm.joints_rightYogaRef(i,end-15:end-12) =  Sm.joints_rightYogaRef(i,end-19:end-16);
	Sm.joints_rightYogaRef(i,end-19:end-16) =  rightArm;
	rightLeg                                =  Sm.joints_rightYogaRef(i,end-5:end);
	Sm.joints_rightYogaRef(i,end-5:end)     =  Sm.joints_rightYogaRef(i,end-11:end-6);
	Sm.joints_rightYogaRef(i,end-11:end-6)  =  rightLeg;
end	 

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

% add harmonic drives reflected inertia
if Config.INCLUDE_HARMONIC_DRIVE_INERTIA
   
    legsMotors_I_m           = legsMotors_I_m + 0.054*1e-4;
    torsoPitchRollMotors_I_m = torsoPitchRollMotors_I_m + 0.054*1e-4;
    torsoYawMotors_I_m       = torsoYawMotors_I_m + 0.021*1e-4;
    armsMotors_I_m           = armsMotors_I_m + 0.021*1e-4; 
end
 
Config.I_m = diag([torsoPitchRollMotors_I_m*ones(2,1);
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
Config.K_ff  = 0.1;

% Config.USE_DES_JOINT_ACC_FOR_MOTORS_INERTIA if true, the desired joints
% accelerations are used for computing the feedforward term in joint
% torques calculations. Not effective if Config.K_ff = 0.
Config.USE_DES_JOINT_ACC_FOR_MOTORS_INERTIA = true;

%% Constraints for QP for balancing

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the circle 
% in each cicle's quadrant
Config.numberOfPoints               = 4;  
Config.forceFrictionCoefficient     = 1/3;    
Config.torsionalFrictionCoefficient = 1/75;
Config.fZmin                        = 10;

% physical size of the foot                             
Config.feet_size                    = [-0.07  0.12  ;    % xMin, xMax
                                       -0.045 0.045 ];   % yMin, yMax
