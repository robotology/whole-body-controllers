function  [w_H_b, constraints, CoM_des, qj_des, impedances, KPCoM, KDCoM, currentState, jointsAndCoMSmoothingTime] = ...
             stateMachineStandup(wrench_rightFoot, wrench_leftFoot, wrench_leftHand, wrench_rightHand, xCoM_0, qj_0, xCoM, l_sole_H_b, l_upper_leg_contact_H_b, t, STANDUP_WITH_HUMAN, Sm, Gain)

    % SATEMACHINESTANDUP state machine for performing standup movements.
    %

    %% --- Initialization ---
    
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;
    persistent CoMprevious;

    if isempty(state) || isempty(tSwitch) || isempty(w_H_fixedLink) || isempty(CoMprevious)
        state         = Sm.stateAt0;
        tSwitch       = 0;
        w_H_fixedLink = l_sole_H_b/l_upper_leg_contact_H_b;
        CoMprevious   = xCoM_0;
    end

    constraints               = [1;1];
    w_H_b                     = eye(4);
    qj_des                    = qj_0;
    CoM_des                   = xCoM_0;
    jointsAndCoMSmoothingTime = Sm.smoothingTimeCoM_Joints(state);

    %% BALANCING ON THE LEGS
    if state == 1 
        
        w_H_b                     =  w_H_fixedLink * l_upper_leg_contact_H_b;
        jointsAndCoMSmoothingTime = Sm.smoothingTimeCoM_Joints(state);
        
        % after tBalancing time, start moving CoM forward. If
        % Config.STANDUP_WITH_HUMAN is enbabled, the robot waits for external 
        % help before lifting up.
        if STANDUP_WITH_HUMAN
            
            if t > Sm.tBalancing && wrench_rightHand(1) > Sm.wrench_thresholdContactRHand(state) && wrench_leftHand(1) > Sm.wrench_thresholdContactLHand(state)
                state = 2;   
                tSwitch = t;
            end
        else
            if t > Sm.tBalancing
                state = 2;                     
                tSwitch  = t;
            end
        end
    end

    %% MOVE COM FORWARD
    if state == 2 
        
        w_H_b                 =  w_H_fixedLink * l_upper_leg_contact_H_b;
        
        % setup new desired position for some joints: remapper
        qj_des([18 19 21 22]) = Sm.joints_standUpPositions(state,[1 2 3 4]);
        qj_des([12 13 15 16]) = Sm.joints_standUpPositions(state,[1 2 3 4]);
        qj_des([8 9 10 11])   = Sm.joints_standUpPositions(state,[5 6 7 8]);
        qj_des([4 5 6 7])     = Sm.joints_standUpPositions(state,[5 6 7 8]);
        qj_des(1)             = Sm.joints_standUpPositions(state,9);
        
        tDelta                    = t-tSwitch;
        CoM_des                   = xCoM_0 + transpose(Sm.CoM_delta(state,:));
        jointsAndCoMSmoothingTime = Sm.smoothingTimeCoM_Joints(state);
        
        if (wrench_rightFoot(3)+wrench_leftFoot(3)) > (Sm.wrench_thresholdContactLFoot(state) + Sm.wrench_thresholdContactRFoot(state)) && tDelta > 1.5
            state           = 3;
            w_H_fixedLink   = w_H_fixedLink * l_upper_leg_contact_H_b/l_sole_H_b;
            tSwitch         = t;
            CoMprevious     = xCoM;
        end     
    end

    %% TWO FEET BALANCING
    if state == 3 
        
        w_H_b                 =  w_H_fixedLink * l_sole_H_b;

        % setup new desired position for some joints: remapper
        qj_des([18 19 21 22]) = Sm.joints_standUpPositions(state,[1 2 3 4]);
        qj_des([12 13 15 16]) = Sm.joints_standUpPositions(state,[1 2 3 4]);
        qj_des([8 9 10 11])   = Sm.joints_standUpPositions(state,[5 6 7 8]);
        qj_des([4 5 6 7])     = Sm.joints_standUpPositions(state,[5 6 7 8]);
        qj_des(1)             = Sm.joints_standUpPositions(state,9);
        
        CoM_des                   = CoMprevious + transpose(Sm.CoM_delta(state,:));
        tDelta                    = t-tSwitch;
        jointsAndCoMSmoothingTime = Sm.smoothingTimeCoM_Joints(state);
        
        if (wrench_leftFoot(3) > Sm.wrench_thresholdContactLFoot(state)) &&  (wrench_rightFoot(3) > Sm.wrench_thresholdContactRFoot(state)) && tDelta > 0.5
            state       = 4;
            tSwitch     = t;
            CoMprevious = xCoM;
        end
    end
    
    %% LIFTING UP
    if state == 4 
        
        w_H_b                 =  w_H_fixedLink * l_sole_H_b;
          
        % setup new desired position for some joints: remapper
        qj_des([18 19 21 22]) = Sm.joints_standUpPositions(state,[1 2 3 4]);
        qj_des([12 13 15 16]) = Sm.joints_standUpPositions(state,[1 2 3 4]);
        qj_des([8 9 10 11])   = Sm.joints_standUpPositions(state,[5 6 7 8]);
        qj_des([4 5 6 7])     = Sm.joints_standUpPositions(state,[5 6 7 8]);
        qj_des(1)             = Sm.joints_standUpPositions(state,9);
        
        CoM_des                   = CoMprevious + transpose(Sm.CoM_delta(state,:));    
        jointsAndCoMSmoothingTime = Sm.smoothingTimeCoM_Joints(state);             
    end
    
    currentState       = state;
    impedances         = Gain.impedances(state,:);
    KPCoM              = Gain.KP_COM(state,:);   
    KDCoM              = Gain.KD_COM(state,:);  
    
end
