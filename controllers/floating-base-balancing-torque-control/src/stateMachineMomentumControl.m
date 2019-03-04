function  [w_H_b, pos_CoM_des, jointPos_des, feetContactStatus, KP_postural_diag, KP_CoM_diag, KD_CoM_diag, state, smoothingTimeJoints, smoothingTimeCoM] = ...
              stateMachineMomentumControl(pos_CoM_0, jointPos_0, pos_CoM_fixed_l_sole, pos_CoM_fixed_r_sole, jointPos, ...
                                          time, wrench_rightFoot, wrench_leftFoot, l_sole_H_b, r_sole_H_b, StateMachine, Gain, Config)

    % STATEMACHINEMOMENTUMCONTROL generates the references for performing
    %                             two demos:
    %
    %                             YOGA: (highly dynamic movements) on one
    %                             foot and two feet;
    %
    %                             COORDINATOR: balancing on two feet while
    %                             performing left-right oscillations.
    
    %% --- Initialization ---

    persistent currentState;
    persistent t_switch;
    persistent w_H_fixedLink;
    persistent yogaMovesetCounter;
    
    if isempty(currentState) 
        
        currentState = StateMachine.initialState;        
    end
    if isempty(t_switch)
    
        t_switch = 0;
    end
    if  isempty(w_H_fixedLink)
        
        w_H_fixedLink = eye(4);
    end
    if isempty(yogaMovesetCounter)
        
        yogaMovesetCounter = 1;
    end
    
    % initialize outputs
    pos_CoM_des       = pos_CoM_0;
    feetContactStatus = [1; 1];
    jointPos_des      = jointPos_0;
    w_H_b             = eye(4);

    %% STATE 1: TWO FEET BALANCING
    if currentState == 1 
        
        w_H_b = w_H_fixedLink * l_sole_H_b;

        % if the demo is COORDINATOR, keep balancing or start to perform
        % left-right movements with the CoM
        if Config.COORDINATOR_DEMO 
            
            if Config.LEFT_RIGHT_MOVEMENTS
                
                if time > Config.noOscillationTime
        
                    Amplitude = Config.amplitudeOfOscillation;
                else
                    Amplitude = 0;
                end
        
                frequency   = Config.frequencyOfOscillation;
                pos_CoM_des = pos_CoM_0 + Amplitude*sin(2*pi*frequency*time)*Config.directionOfOscillation;        
            end        
        else          
            % after tBalancing time start moving the weight to the left
            if time > StateMachine.tBalancing
         
               currentState = 2;
           
               if StateMachine.demoStartsOnRightSupport
               
                    w_H_fixedLink = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
                    currentState  = 8;
               end           
            end
        end
    end

    %% STATE 2: TRANSITION TO THE LEFT FOOT
    if currentState == 2 
        
        w_H_b = w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        pos_CoM_des       = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
        
        fixed_link_CoMDes = w_H_fixedLink\[pos_CoM_des; 1];
        pos_CoM_error     = fixed_link_CoMDes(1:3) - pos_CoM_fixed_l_sole(1:3);
        jointPos_des      = StateMachine.joints_references(currentState,:)';
        
        if norm(pos_CoM_error(2)) < StateMachine.CoM_threshold && wrench_rightFoot(3) < StateMachine.wrench_thresholdContactOff
            
           currentState = 3; 
           t_switch     = time;
        end
    end

    %% STATE 3: LEFT FOOT BALANCING 
    if currentState == 3 
        
        w_H_b = w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        pos_CoM_des       = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
 
        % right foot is no longer in contact
        feetContactStatus = [1; 0];

        jointPos_des      = StateMachine.joints_references(currentState,:)'; 

        if time > (t_switch + StateMachine.tBalancingBeforeYoga) 
            
            currentState = 4;
            t_switch     = time;
            
            if StateMachine.skipYoga
                
                currentState = 5;
            end
        end
    end
    
    %% STATE 4: YOGA LEFT FOOT
    if currentState == 4 
        
        w_H_b = w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        pos_CoM_des        = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
        feetContactStatus  = [1; 0]; 
        jointPos_des       = StateMachine.joints_references(currentState,:)';

        % iterate over the yoga positions
        for i = 1: size(StateMachine.joints_leftYogaRef,1)-1
            
            % positions for the yoga movements
            if time > (StateMachine.joints_leftYogaRef(i,1) + t_switch) && time <= (StateMachine.joints_leftYogaRef(i+1,1)+ t_switch)
                
                jointPos_des = StateMachine.joints_leftYogaRef(i,2:end)';
            end 
        end
        if time > (StateMachine.joints_leftYogaRef(end,1) + t_switch)
            
            jointPos_des = StateMachine.joints_leftYogaRef(end,2:end)';
            
            % if StateMachine.yogaCounter > 1, yoga in the loop. Repeat the Yoga movements N times
            if time > (StateMachine.joints_leftYogaRef(end,1) + t_switch + StateMachine.jointsSmoothingTime(currentState) + StateMachine.joints_pauseBetweenYogaMoves)
                 
                t_switch           = time;
                yogaMovesetCounter = yogaMovesetCounter +1;
                
                % if the robot repeated the Yoga moveset for the number of
                % times required by the user, then exit the loop
                if yogaMovesetCounter > StateMachine.yogaCounter
                   
                    currentState  = 5;
                end
            end
        end        
    end
    
    %% STATE 5: PREPARING FOR SWITCHING
    if currentState == 5 
        
        w_H_b = w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        pos_CoM_des        = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
        feetContactStatus  = [1; 0]; 
        jointPos_des       = StateMachine.joints_references(currentState,:)';

        jointPos_errorRLeg = jointPos(end-5:end)    - jointPos_des(end-5:end);      
        jointPos_errorLLeg = jointPos(end-11:end-6) - jointPos_des(end-11:end-6);
            
        if norm(jointPos_errorRLeg)*180/pi < StateMachine.joints_thresholdNotInContact && norm(jointPos_errorLLeg)*180/pi < StateMachine.joints_thresholdInContact
            
            currentState   = 6;
            t_switch       = time;
        end
    end
    
    %% STATE 6: LOOKING FOR A CONTACT
    if currentState == 6 
        
        w_H_b = w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        pos_CoM_des       = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
        feetContactStatus = [1; 0]; 
        jointPos_des      = StateMachine.joints_references(currentState,:)';

        if wrench_rightFoot(3) > StateMachine.wrench_thresholdContactOn
            
            currentState  = 7;
            t_switch      = time;
        end
    end
    
    %% STATE 7: TRANSITION TO INITIAL POSITION
    if currentState == 7 
        
        w_H_b = w_H_fixedLink * l_sole_H_b;
        
        pos_CoM_des        = pos_CoM_0 + StateMachine.CoM_delta(currentState,:)';        
        
        % right foot is in contact
        feetContactStatus  = [1; 1]; 

        if norm(pos_CoM_fixed_l_sole(1:2) -pos_CoM_des(1:2)) < 10*StateMachine.CoM_threshold && StateMachine.yogaAlsoOnRightFoot && time > t_switch + StateMachine.tBalancing
            
            w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
            currentState    = 8;
            t_switch        = time;
        end
    end
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%   
%%                           YOGA RIGHT FOOT                             %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

    %% STATE 8: TRANSITION TO THE RIGHT FOOT
    if currentState == 8 

        w_H_b = w_H_fixedLink * r_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the right foot (r_sole) plus a
        % configurable delta
        pos_CoM_des           = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
 
        feetContactStatus     = [1; 1]; 
        fixed_link_CoMDes     = w_H_fixedLink\[pos_CoM_des;1];
        pos_CoM_error         = fixed_link_CoMDes(1:3) - pos_CoM_fixed_r_sole(1:3);
        jointPos_des          = StateMachine.joints_references(currentState,:)';

        if norm(pos_CoM_error(2)) < StateMachine.CoM_threshold  && wrench_leftFoot(3) < StateMachine.wrench_thresholdContactOff
            
           currentState = 9; 
           t_switch     = time;
        end
    end
    
     %% STATE 9: RIGHT FOOT BALANCING 
    if currentState == 9
        
        w_H_b = w_H_fixedLink * r_sole_H_b;
        
        % left foot is no longer in contact
        feetContactStatus = [0; 1]; 

        pos_CoM_des       = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
        jointPos_des      = StateMachine.joints_references(currentState,:)';
        
        if time > (t_switch + StateMachine.tBalancingBeforeYoga)
            
            currentState  = 10;
            t_switch      = time;
            
            if StateMachine.skipYoga
                
                currentState = 11;
            end
        end
    end
    
    %% STATE 10: YOGA RIGHT FOOT
    if currentState == 10 
      
        w_H_b = w_H_fixedLink*r_sole_H_b;
        
        feetContactStatus = [0; 1]; 
        pos_CoM_des       = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
        jointPos_des      = StateMachine.joints_references(currentState,:)';

        % iterate over the yoga positions
        for i = 1: size(StateMachine.joints_rightYogaRef,1)-1
            
            % positions for the yoga movements
            if time > (StateMachine.joints_rightYogaRef(i,1) + t_switch) && time <= (StateMachine.joints_rightYogaRef(i+1,1)+ t_switch)
                
                jointPos_des = StateMachine.joints_rightYogaRef(i,2:end)';
            end
        end     
        if time > (StateMachine.joints_rightYogaRef(end,1) + t_switch)
            
            jointPos_des = StateMachine.joints_rightYogaRef(end,2:end)';
            
            % if StateMachine.yogaCounter > 1, yoga in the loop. Repeat the Yoga movements N times
            if time > (StateMachine.joints_rightYogaRef(end,1) + t_switch + StateMachine.jointsSmoothingTime(currentState) + StateMachine.joints_pauseBetweenYogaMoves)
            
                t_switch           = time;
                yogaMovesetCounter = yogaMovesetCounter +1;
                
                % if the robot repeated the Yoga moveset for the number of
                % times required by the user, then exit the loop
                if yogaMovesetCounter > StateMachine.yogaCounter
                   
                    currentState = 11;
                end
            end
        end
    end
    
    %% STATE 11: PREPARING FOR SWITCHING
    if currentState == 11 
        
        w_H_b = w_H_fixedLink * r_sole_H_b;
        
        feetContactStatus  = [0; 1];
        pos_CoM_des        = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
        jointPos_des       = StateMachine.joints_references(currentState,:)'; 

        jointPos_errorRLeg = jointPos(end-5:end) -jointPos_des(end-5:end);
        jointPos_errorLLeg = jointPos(end-11:end-6) -jointPos_des(end-11:end-6);
            
        if norm(jointPos_errorRLeg)*180/pi < StateMachine.joints_thresholdInContact && norm(jointPos_errorLLeg)*180/pi < StateMachine.joints_thresholdNotInContact
            
            currentState = 12;
            t_switch     = time;
        end
    end
    
    %% STATE 12: LOOKING FOR A CONTACT
    if currentState == 12

        w_H_b = w_H_fixedLink  * r_sole_H_b;
               
        feetContactStatus = [0; 1];
        pos_CoM_des       = [w_H_fixedLink(1:2,4); pos_CoM_0(3)] + StateMachine.CoM_delta(currentState,:)';         
        jointPos_des      = StateMachine.joints_references(currentState,:)';

        if wrench_leftFoot(3) > StateMachine.wrench_thresholdContactOn
            
            currentState  = 13;
            t_switch      = time;
        end
    end
    
    %% STATE 13: TRANSITION TO INITIAL POSITION
    if currentState == 13
        
        w_H_b = w_H_fixedLink * r_sole_H_b;
        
        % left foot is in contact
        feetContactStatus = [1; 1];  
        
        if (time -t_switch) > StateMachine.tBalancing 
            
           if StateMachine.twoFeetYogaInLoop
               
              currentState  = 2; 
              w_H_fixedLink = w_H_fixedLink*r_sole_H_b/l_sole_H_b;
              
              if StateMachine.demoStartsOnRightSupport
                  
                 currentState  = 8;           
                 w_H_fixedLink = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
              end
           end
        end
    end
    
    % Update joints and CoM smoothing time
    if currentState == 4 || currentState == 10
            
        % during the yoga, reduce the time necessary for the joints 
        % reference to converge to the next position
        smoothingTimeJoints = StateMachine.scaleFactorSmoothingTime*StateMachine.jointsSmoothingTime(currentState);
    else
        smoothingTimeJoints = StateMachine.jointsSmoothingTime(currentState);
    end
    
    smoothingTimeCoM = StateMachine.CoMSmoothingTime(currentState);
    
    % update gain matrices
    KP_postural_diag = Gain.KP_postural(currentState,:);
    KP_CoM_diag      = Gain.KP_CoM(currentState,:);   
    KD_CoM_diag      = Gain.KD_CoM(currentState,:); 
    
    % update current state
    state            = currentState;  
end