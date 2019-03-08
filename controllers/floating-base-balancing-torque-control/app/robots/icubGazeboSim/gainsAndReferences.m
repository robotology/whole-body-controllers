% GAINSANDREFERENCES compute gains matrices, references and other control
%                    related quantities for each state of the state machine.

%% --- Initialization ---
  
% CoM gains
Gain.KP_CoM = [10  50  10   % state ==  1  TWO FEET BALANCING
               10  50  10   % state ==  2  COM TRANSITION TO LEFT 
               10  50  10   % state ==  3  LEFT FOOT BALANCING
               10  50  10   % state ==  4  YOGA LEFT FOOT 
               10  50  10   % state ==  5  PREPARING FOR SWITCHING 
               10  50  10   % state ==  6  LOOKING FOR CONTACT
               10  50  10   % state ==  7  TRANSITION TO INITIAL POSITION 
               10  50  10   % state ==  8  COM TRANSITION TO RIGHT FOOT
               10  50  10   % state ==  9  RIGHT FOOT BALANCING
               10  50  10   % state == 10  YOGA RIGHT FOOT 
               10  50  10   % state == 11  PREPARING FOR SWITCHING 
               10  50  10   % state == 12  LOOKING FOR CONTACT
               10  50  10]; % state == 13  TRANSITION TO INITIAL POSITION

Gain.KD_CoM = 2*sqrt(Gain.KP_CoM);

% Angular momentum gains
Gain.KI_AngularMomentum = 0.25 ;
Gain.KP_AngularMomentum = 2*sqrt(Gain.KI_AngularMomentum);

% Postural task gains
%                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
Gain.KP_postural = [10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  1  TWO FEET BALANCING
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  2  COM TRANSITION TO LEFT 
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  3  LEFT FOOT BALANCING
                    30   30   30, 10   10    10   10, 10   10    10   10, 50   50  250   200     50  50, 50   50   50    50     50  50  % state ==  4  YOGA LEFT FOOT 
                    30   30   30, 10   10    10   10, 10   10    10   10, 30   50  300    60     50  50, 30   50   30    60     50  50  % state ==  5  PREPARING FOR SWITCHING 
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  6  LOOKING FOR CONTACT
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  7  TRANSITION TO INITIAL POSITION 
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  8  COM TRANSITION TO RIGHT FOOT
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  9  RIGHT FOOT BALANCING
                    30   30   30, 10   10    10   10, 10   10    10   10, 50   50   50    50     50  50, 50   50  250   200     50  50  % state == 10  YOGA RIGHT FOOT 
                    30   30   30, 10   10    10   10, 10   10    10   10, 30   50   30    60     50  50, 30   50  300    60     50  50  % state == 11  PREPARING FOR SWITCHING 
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state == 12  LOOKING FOR CONTACT
                    10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50];% state == 13  TRANSITION TO INITIAL POSITION
 
Gain.KD_postural = 0*sqrt(Gain.KP_postural(1,:));  

%% Smoothing times

% Smoothing time gain scheduling
Config.SmoothingTimeGainScheduling = 2;

% Smoothing time CoM references
StateMachine.CoMSmoothingTime    = [5;   %% state ==  1  TWO FEET BALANCING
                                    5;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    3;   %% state ==  3  LEFT FOOT BALANCING 
                                    4;   %% state ==  4  YOGA LEFT FOOT
                                    5;   %% state ==  5  PREPARING FOR SWITCHING
                                    5;   %% state ==  6  LOOKING FOR CONTACT 
                                    4;   %% state ==  7  TRANSITION INIT POSITION
                                    5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                                    3;   %% state ==  9  RIGHT FOOT BALANCING 
                                    4;   %% state == 10  YOGA RIGHT FOOT
                                    5;   %% state == 11  PREPARING FOR SWITCHING
                                    5;   %% state == 12  LOOKING FOR CONTACT 
                                    4];  %% state == 13  TRANSITION INIT POSITION

% Smoothing time for joints references 
StateMachine.jointsSmoothingTime = [5;   %% state ==  1  TWO FEET BALANCING
                                    5;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    3;   %% state ==  3  LEFT FOOT BALANCING 
                                    4;   %% state ==  4  YOGA LEFT FOOT
                                    5;   %% state ==  5  PREPARING FOR SWITCHING
                                    5;   %% state ==  6  LOOKING FOR CONTACT 
                                    4;   %% state ==  7  TRANSITION INIT POSITION
                                    5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                                    3;   %% state ==  9  RIGHT FOOT BALANCING 
                                    4;   %% state == 10  YOGA RIGHT FOOT
                                    5;   %% state == 11  PREPARING FOR SWITCHING
                                    5;   %% state == 12  LOOKING FOR CONTACT 
                                    4];  %% state == 13  TRANSITION INIT POSITION

% scale factor smoothing time multiplies the smoothing factor during the
% Yoga (state 4 and 10). The purpose is to reduce the time necessary for 
% the reference to converge to the next position, but without changing also
% the valuse stored in Sm.joints_leftYogaRef/Sm.joints_rightYogaRef
StateMachine.scaleFactorSmoothingTime = 0.9;
                                
%% CoM delta

% To be summed to the reference CoM position
StateMachine.CoM_delta  = [% THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                           0.0,  0.00,  0.0;   %% NOT USED
                           0.0,  0.01,  0.0;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                           0.0,  0.00,  0.0;   %% state ==  3  LEFT FOOT BALANCING 
                           0.0,  0.01,  0.0;   %% state ==  4  YOGA LEFT FOOT
                           0.0,  0.00,  0.0;   %% state ==  5  PREPARING FOR SWITCHING
                           0.0, -0.09,  0.0;   %% state ==  6  LOOKING FOR CONTACT 
                           0.0,  0.00,  0.0;   %% state ==  7  TWO FEET BALANCING
                           % THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE RIGHT FOOT
                           0.0, -0.01,  0.0;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                           0.0,  0.00,  0.0;   %% state ==  9  RIGHT FOOT BALANCING 
                           0.0, -0.00,  0.0;   %% state == 10  YOGA RIGHT FOOT
                           0.0,  0.00,  0.0;   %% state == 11  PREPARING FOR SWITCHING
                           0.0,  0.09,  0.0;   %% state == 12  LOOKING FOR CONTACT 
                           0.0,  0.00,  0.0];  %% NOT USED

%% Joint references
StateMachine.joints_references = [ zeros(1,ROBOT_DOF);                                %% THIS REFERENCE IS IGNORED 
                                 [-0.0348,0.0779,0.0429, ...                          %% state == 2  COM TRANSITION TO LEFT 
                                  -0.1493,0.8580,0.2437,0.8710, ...                   %
                                  -0.1493,0.8580,0.2437,0.8710, ...                   %
                                  -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...   %  
                                   0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     %  
                                 [ 0.0864,0.0258,0.0152, ...                          %% state == 3  LEFT FOOT BALANCING
                                   0.1253,0.8135,0.3051,0.7928, ...                   %    
                                   0.0563,0.6789,0.3340,0.6214, ...                   %
                                  -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...   %  
                                   0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     % 
                                   zeros(1,ROBOT_DOF);                                %% THIS REFERENCE IS IGNORED 
                                 [-0.0348,0.0779,0.0429, ...                          %% state == 5  PREPARING FOR SWITCHING
                                  -0.1493,0.8580,0.2437,0.8710, ...                   %
                                  -0.1493,0.8580,0.2437,0.8710, ...                   %
                                  -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...   %  
                                   0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     % 
                                 [ 0.0864,0.0258,0.0152, ...                          %% state == 6  LOOKING FOR CONTACT
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

% YOGA MOVESET (joint references during state 4 and 10)
q1 =        [-0.0790,0.2279, 0.4519, ...
             -1.1621,0.6663, 0.4919, 0.9947, ... 
             -1.0717,1.2904,-0.2447, 1.0948, ...
              0.2092,0.2060, 0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3484,0.4008,-0.0004,-0.3672,-0.1060,-0.0875];

q2 =        [-0.0790,0.2279, 0.4519, ...
             -1.1621,0.6663, 0.4965, 0.9947, ...
             -1.0717,1.2904,-0.2493, 1.0948, ...
              0.2092,0.2060, 0.0006,-0.1741,-0.1044,0.0700, ... 
              0.3714,0.9599, 1.3253,-1.6594,-0.1060,-0.0614];
          
q3 =        [-0.0852,-0.4273,0.0821,...
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
                           
StateMachine.joints_leftYogaRef  = [ 0,                                    q1;
                                     1*StateMachine.jointsSmoothingTime(4),q2;
                                     2*StateMachine.jointsSmoothingTime(4),q3;
                                     3*StateMachine.jointsSmoothingTime(4),q4;
                                     4*StateMachine.jointsSmoothingTime(4),q5;
                                     5*StateMachine.jointsSmoothingTime(4),q6;
                                     6*StateMachine.jointsSmoothingTime(4),q7;
                                     7*StateMachine.jointsSmoothingTime(4),q8];
                 
StateMachine.joints_rightYogaRef      = StateMachine.joints_leftYogaRef;
StateMachine.joints_rightYogaRef(:,1) = [0;
                                         1*StateMachine.jointsSmoothingTime(10);
                                         2*StateMachine.jointsSmoothingTime(10);
                                         3*StateMachine.jointsSmoothingTime(10);
                                         4*StateMachine.jointsSmoothingTime(10);
                                         5*StateMachine.jointsSmoothingTime(10);
                                         6*StateMachine.jointsSmoothingTime(10);
                                         7*StateMachine.jointsSmoothingTime(10)];

% MIRROR YOGA LEFT MOVESET FOR RIGHT YOGA					 
for i = 1:size(StateMachine.joints_rightYogaRef,1)	
    
	StateMachine.joints_rightYogaRef(i,2:4)           = [StateMachine.joints_rightYogaRef(i,2) -StateMachine.joints_rightYogaRef(i,3) -StateMachine.joints_rightYogaRef(i,4)];
	rightArm                                          =  StateMachine.joints_rightYogaRef(i,end-15:end-12);
	StateMachine.joints_rightYogaRef(i,end-15:end-12) =  StateMachine.joints_rightYogaRef(i,end-19:end-16);
	StateMachine.joints_rightYogaRef(i,end-19:end-16) =  rightArm;
	rightLeg                                          =  StateMachine.joints_rightYogaRef(i,end-5:end);
	StateMachine.joints_rightYogaRef(i,end-5:end)     =  StateMachine.joints_rightYogaRef(i,end-11:end-6);
	StateMachine.joints_rightYogaRef(i,end-11:end-6)  =  rightLeg;
end	 

%% References for CoM trajectory (COORDINATOR DEMO ONLY)

% that the robot waits before starting the left-and-right 
Config.noOscillationTime       = 0;   
Config.directionOfOscillation  = [0;1;0];
Config.amplitudeOfOscillation  = 0.02; % [m]  
Config.frequencyOfOscillation  = 0.2;  % [Hz]