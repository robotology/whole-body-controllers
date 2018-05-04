function  [w_H_b, CoM_des, s_des, feet_in_contact, Kp_joints, Kp_CoM, Kd_CoM, currentState, smoothingTime_CoM_Joints] = ...
            stateMachineYoga(CoM_0, s_0, l_sole_CoM, r_sole_CoM, s, t, wrench_rightFoot, wrench_leftFoot, l_sole_H_b, r_sole_H_b, Sm, Gain)
    
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;

    if isempty(state) || isempty(tSwitch) || isempty(w_H_fixedLink) 
        state         = Sm.stateAt0;
        tSwitch       = 0;
        w_H_fixedLink = eye(4);
    end
    
    CoM_des         = CoM_0;
    feet_in_contact = [1; 1];
    s_des           = s_0;
    w_H_b           = eye(4);
    Kp_joints       = Gain.Kp_joints(1,:);
    Kp_CoM          = Gain.Kp_CoM(1,:);   
    Kd_CoM          = Gain.Kd_CoM(1,:);   

    %% TWO FEET BALANCING
    if state == 1 
        
        w_H_b    =  w_H_fixedLink * l_sole_H_b;

        % after tBalancing time start moving weight to the left
        if t > Sm.tBalancing && ~Sm.demoOnlyBalancing 
         
           state = 2;
           
           if Sm.demoStartsOnRightSupport
               
                w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
                state = 8;
           end           
        end
    end

    %% TRANSITION TO THE LEFT FOOT
    if state == 2 
        
        w_H_b    =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoM_des     = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
        
        Kp_joints   = Gain.Kp_joints(state,:);
        Kp_CoM      = Gain.Kp_CoM(state,:);   
        Kd_CoM      = Gain.Kd_CoM(state,:);   

        fixed_link_CoMDes = w_H_fixedLink\[CoM_des;1];
        CoM_Error         = fixed_link_CoMDes(1:3) -l_sole_CoM(1:3);
        s_des             = Sm.joints_references(state,:)';
        
        if norm(CoM_Error(2)) < Sm.CoM_threshold && wrench_rightFoot(3) < Sm.wrench_thresholdContactOff
           state   = 3; 
           tSwitch = t;
        end
    end

    %% LEFT FOOT BALANCING 
    if state == 3 
        
        w_H_b       =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoM_des     = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
 
        % right foot is no longer a constraints
        feet_in_contact = [1; 0];

        s_des        = Sm.joints_references(state,:)';
        Kp_joints    = Gain.Kp_joints(state,:);
        Kp_CoM       = Gain.Kp_CoM(state,:);   
        Kd_CoM       = Gain.Kd_CoM(state,:);   

        if t > (tSwitch + Sm.tBalancingBeforeYoga) 
            
            state   = 4;
            tSwitch = t;
            
            if Sm.skipYoga
                state = 5;
            end
        end
    end
    
    %% YOGA LEFT FOOT
    if state == 4 
        
        w_H_b        =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoM_des      = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
        
        feet_in_contact  = [1; 0]; 

        s_des         = Sm.joints_references(state,:)';
        Kp_joints     = Gain.Kp_joints(state,:);
        Kp_CoM        = Gain.Kp_CoM(state,:);   
        Kd_CoM        = Gain.Kd_CoM(state,:);   

        for i = 1: size(Sm.joints_leftYogaRef,1)-1
            
            if t > (Sm.joints_leftYogaRef(i,1) + tSwitch) && t <= (Sm.joints_leftYogaRef(i+1,1)+ tSwitch)
                
                s_des = Sm.joints_leftYogaRef(i,2:end)';
            end
        end
        if t > (Sm.joints_leftYogaRef(end,1) + tSwitch) 
            
            s_des = Sm.joints_leftYogaRef(end,2:end)';
            
            if t > (Sm.joints_leftYogaRef(end,1) + tSwitch + Sm.smoothingTime_CoM_Joints(state) + Sm.joints_pauseBetweenYogaMoves)
                state   = 5;
                tSwitch = t;
            end
        end
    end
    
    %% PREPARING FOR SWITCHING
    if state == 5 
        
        w_H_b       = w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoM_des     = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
        
        feet_in_contact = [1; 0]; 

        s_des        = Sm.joints_references(state,:)';
        Kp_joints    = Gain.Kp_joints(state,:);
        Kp_CoM       = Gain.Kp_CoM(state,:);   
        Kd_CoM       = Gain.Kd_CoM(state,:);   

        qj_errorRLeg  = s(end-5:end)    -s_des(end-5:end);      
        qj_errorLLeg  = s(end-11:end-6) -s_des(end-11:end-6);
            
        if norm(qj_errorRLeg)*180/pi < Sm.joints_thresholdNotInContact && norm(qj_errorLLeg)*180/pi < Sm.joints_thresholdInContact
            
            state   = 6;
            tSwitch = t;
        end
    end
    
    %% LOOKING FOR A CONTACT
    if state == 6 
        
        w_H_b        =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoM_des      = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
        
        feet_in_contact = [1; 0]; 

        s_des        = Sm.joints_references(state,:)';
        Kp_joints    = Gain.Kp_joints(state,:);
        Kp_CoM       = Gain.Kp_CoM(state,:);   
        Kd_CoM       = Gain.Kd_CoM(state,:);   

        if wrench_rightFoot(3) > Sm.wrench_thresholdContactOn
            
            state   = 7;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 7 
        
        w_H_b        =  w_H_fixedLink * l_sole_H_b;
     
        CoM_des      = CoM_0 + Sm.CoM_delta(state,:)';        
    
        feet_in_contact = [1; 1]; 
        Kp_joints       = Gain.Kp_joints(state,:);
        Kp_CoM          = Gain.Kp_CoM(state,:);   
        Kd_CoM          = Gain.Kd_CoM(state,:); 
        
        if norm(l_sole_CoM(1:2)-CoM_des(1:2)) < 10*Sm.CoM_threshold && Sm.yogaAlsoOnRightFoot && t > tSwitch + Sm.tBalancing
            
            w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
            state           = 8;
            tSwitch         = t;
        end
    end
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           

    %% TRANSITION TO THE RIGHT FOOT
    if state == 8 
        
        feet_in_contact = [1; 1]; 
        w_H_b           =  w_H_fixedLink * r_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoM_des           = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
 
        fixed_link_CoMDes = w_H_fixedLink\[CoM_des;1];
        CoM_Error         = fixed_link_CoMDes(1:3) - r_sole_CoM(1:3);
        
        Kp_joints    = Gain.Kp_joints(state,:);
        Kp_CoM       = Gain.Kp_CoM(state,:);   
        Kd_CoM       = Gain.Kd_CoM(state,:);   
        s_des        = Sm.joints_references(state,:)';

        if norm(CoM_Error(2)) < Sm.CoM_threshold  && wrench_leftFoot(3) < Sm.wrench_thresholdContactOff
            
           state   = 9; 
           tSwitch = t;
        end
    end
    
     %% RIGHT FOOT BALANCING 
    if state == 9
        
        % left foot is no longer a constraints
        feet_in_contact = [0; 1]; 
        w_H_b           =  w_H_fixedLink * r_sole_H_b;

        CoM_des      = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
        s_des        = Sm.joints_references(state,:)';
        Kp_joints    = Gain.Kp_joints(state,:);
        Kp_CoM       = Gain.Kp_CoM(state,:);   
        Kd_CoM       = Gain.Kd_CoM(state,:);
        
        if t > (tSwitch + Sm.tBalancingBeforeYoga)
            
            state   = 10;
            tSwitch = t;
            
            if Sm.skipYoga
                state = 11;
            end
        end
    end
    
    %% YOGA RIGHT FOOT
    if state == 10 
        
        feet_in_contact = [0; 1]; 
        w_H_b       =  w_H_fixedLink*r_sole_H_b;

        CoM_des     = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
        s_des       = Sm.joints_references(state,:)';
        Kp_joints   = Gain.Kp_joints(state,:);
        Kp_CoM      = Gain.Kp_CoM(state,:);   
        Kd_CoM      = Gain.Kd_CoM(state,:);   

        for i = 1: size(Sm.joints_rightYogaRef,1)-1
            
            if t > (Sm.joints_rightYogaRef(i,1) + tSwitch) && t <= (Sm.joints_rightYogaRef(i+1,1)+ tSwitch)
                
                s_des = Sm.joints_rightYogaRef(i,2:end)';
            end
        end
        if t > Sm.joints_rightYogaRef(end,1) + tSwitch 
            
            s_des = Sm.joints_rightYogaRef(end,2:end)';
            
            if t > (Sm.joints_rightYogaRef(end,1) + tSwitch + Sm.smoothingTime_CoM_Joints(state) + Sm.joints_pauseBetweenYogaMoves) 
            
                state   = 11;
                tSwitch = t;
            end
        end
    end
    
    %% PREPARING FOR SWITCHING
    if state == 11 
        
        feet_in_contact = [0; 1];
        w_H_b           =  w_H_fixedLink * r_sole_H_b;

        CoM_des         = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
        s_des           = Sm.joints_references(state,:)';
        Kp_joints       = Gain.Kp_joints(state,:);
        Kp_CoM          = Gain.Kp_CoM(state,:);   
        Kd_CoM          = Gain.Kd_CoM(state,:);   

        qj_errorRLeg    = s(end-5:end)-s_des(end-5:end);
        qj_errorLLeg    = s(end-11:end-6)-s_des(end-11:end-6);
            
        if norm(qj_errorRLeg)*180/pi < Sm.joints_thresholdInContact && norm(qj_errorLLeg)*180/pi < Sm.joints_thresholdNotInContact
            state   = 12;
            tSwitch = t;
        end
    end
    
    %% LOOKING FOR A CONTACT
    if state == 12
        
        feet_in_contact = [0; 1];
        w_H_b           =  w_H_fixedLink  * r_sole_H_b;

        CoM_des         = [w_H_fixedLink(1:2,4);CoM_0(3)] + Sm.CoM_delta(state,:)';         
        s_des           = Sm.joints_references(state,:)';
        Kp_joints       = Gain.Kp_joints(state,:);
        Kp_CoM          = Gain.Kp_CoM(state,:);   
        Kd_CoM          = Gain.Kd_CoM(state,:);   

        if wrench_leftFoot(3) > Sm.wrench_thresholdContactOn
            
            state   = 13;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 13
        
        w_H_b           =  w_H_fixedLink * r_sole_H_b;
        feet_in_contact = [1; 1]; 
        Kp_joints       = Gain.Kp_joints(state,:);
        Kp_CoM          = Gain.Kp_CoM(state,:);   
        Kd_CoM          = Gain.Kd_CoM(state,:);   
        
        if t - tSwitch > Sm.tBalancing 
            
           if Sm.yogaInLoop
               
              state = 2; 
              w_H_fixedLink = w_H_fixedLink*r_sole_H_b/l_sole_H_b;
              
              if Sm.demoStartsOnRightSupport
                 state = 8;           
                 w_H_fixedLink = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
              end
           end
        end
    end 
    
    %% Update parameters
    currentState = state;
    smoothingTime_CoM_Joints = Sm.smoothingTime_CoM_Joints(state);
    
end
