% STATEMACHINEWALKING computes robot state and references of the walking
%                     with MPC demo.
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b, leftIsFixed_out, useFeetPIDs] = stateMachineWalking ...
%             (s_0, pos_vel_acc_CoM_des_walking, s_des_walking, feetInContact_walking, b_H_l, b_H_r, w_walking_H_LFoot, w_walking_H_RFoot, w_H_rot_task_0, w_walking_H_b_initial, ...
%              LFoot_is_fixed, LFoot_wrench, RFoot_wrench, Config, Sat)
%
% INPUT:  - s_0 = [ROBOT_DOF * 1] initial joints positions
%         - pos_vel_acc_CoM_des_walking = [3 * 3] CoM references from MPC
%         - s_des_walking = [ROBOT_DOF * 1] joints references from MPC
%         - feetInContact_walking  = [2 * 1] active contacts with MPC
%         - b_H_l = [4 * 4] base to LFoot transform
%         - b_H_r = [4 * 4] base to RFoot transform
%         - w_walking_H_LFoot = [4 * 4] world to LFoot transform
%         - w_walking_H_RFoot = [4 * 4] world to RFoot transform
%         - w_H_rot_task_initial= [4 * 4] world to rot task transform in the initial configuration
%         - w_walking_H_b_initial = [4 * 4] base to world (MPC) transform in the initial configuration
%         - LFoot_is_fixed = boolean for checking if the left foot is fixed on ground
%         - LFoot_wrench = [6 * 1] external forces and moments acting on the left foot
%         - RFoot_wrench = [6 * 1] external forces and moments acting on the right foot
%         - Config = user defined configuration
%         - Sat = a structure containing user defined saturation parameters
%
% OUTPUT: - state = current state of state machine
%         - references_CoM = [3 * 3] desired CoM position, velocity and acceleration
%         - references_LFoot = [6 * 5] desired LFoot pose, velocity
%                              and acceleration. NOTE that the format
%                              is: [pos, linear_vel, linear_acc, zeros(3,2);
%                                   Rotation, angular_vel, angular_acc] 
%         - references_RFoot = [6 * 5] desired RFoot pose, velocity and acceleration
%         - references_RotTask = [3 * 5] desired rotational task link orientation, angular velocity and acceleration.
%                                NOTE that the format is: [Rotation, angular_vel, angular_acc]     
%         - feetInContact = [2 * 1] feet in contact
%         - references_s = [ROBOT_DOF * 3] desired joint positions, velocities and accelerations
%         - w_H_b = [4 * 4] world to base transform
%         - leftIsFixed_out = boolean for checking if the left foot is fixed on ground
%         - useFeetPIDs = [2 *1] boolean for feet control activation
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b, w_H_bWithImu, leftIsFixed_out, useFeetPIDs, omega_world] = stateMachineWalking ...
             (s_0, pos_vel_acc_CoM_des_walking, s_des_walking, feetInContact_walking, b_H_l, b_H_r, w_walking_H_LFoot, w_walking_H_RFoot, w_H_rot_task_initial, b_H_RFoot_desired, ...
              LFoot_is_fixed, LFoot_wrench, RFoot_wrench, waistImu, Config, Sat)
         
    % State selector:
    %
    % state = 1 two feet blancing
    % state = 2 left foot balancing
    % state = 3 right foot balancing
    %
    persistent currentState
    persistent leftAsFixedLink
    persistent w_H_fixedLink
    persistent time
    persistent tState
    persistent w_R_wImu
    
    if isempty(tState)
        tState = 0;
    end
    
    if isempty(currentState)
        
        currentState = 1;
    end
    if isempty(leftAsFixedLink)
       
        leftAsFixedLink = ~(round(LFoot_is_fixed) > 0.9); %to avoid switching fixed link at startup
    end
    if isempty(w_H_fixedLink)
       
        if leftAsFixedLink
        
            w_H_fixedLink = w_walking_H_LFoot;
        else
            w_H_fixedLink = w_walking_H_RFoot;
        end
    end 
    if isempty(time)
       
        time = 0.0;
    end
    
    
    b_R_imu = rotz(-1.57079632679)*roty(0)*rotx(-2.09439521059);

    if isempty(w_R_wImu)
        wImu_R_imu_0      = rotz(deg2rad(waistImu(3)))*roty(deg2rad(waistImu(2)))*rotx(deg2rad(waistImu(1)));
        w_H_b_0           = w_walking_H_RFoot/b_H_RFoot_desired;
        w_R_b_0           = w_H_b_0(1:3,1:3);
        w_R_imu_0         = w_R_b_0*b_R_imu;
        w_R_wImu          = w_R_imu_0/(wImu_R_imu_0);
    end
    
    % base to imu rotation (fixed) and initial base to world rotation
    % (fixed)
        
    wImu_R_imu   = rotz(deg2rad(waistImu(3)))*roty(deg2rad(waistImu(2)))*rotx(deg2rad(waistImu(1)));
    w_R_imu      = w_R_wImu*wImu_R_imu;
    w_R_b        = w_R_imu/b_R_imu;
    w_H_bWithImu = eye(4);
     
    % Active contacts treshold
    toll = Sat.toll_feetInContact;
    
    %% Use feet PIDs for controlling feet position and orientation
    useFeetPIDs = [1, 1];   
    
    %% State Selector
    if sum(feetInContact_walking) > (2-toll) && LFoot_wrench(3) > Config.threshold_contact_twoFeet && RFoot_wrench(3) > Config.threshold_contact_twoFeet && (currentState ~= 1)
   
       currentState  = 1;
       tState        = 0;
       
    elseif feetInContact_walking(1) > (1-toll) && feetInContact_walking(2) < toll && RFoot_wrench(3) < Config.threshold_contact_off && currentState ~= 2
        
        currentState = 2;
        tState       = 0;
       
    elseif feetInContact_walking(2) > (1-toll) && feetInContact_walking(1) < toll && LFoot_wrench(3) < Config.threshold_contact_off && currentState ~= 3    
        
        currentState = 3;        
        tState       = 0;
      
    end
    
    %% Feet in contact selector    
    if currentState == 2
        
        feetInContact = [1,0];
    
    elseif currentState == 3
       
        feetInContact = [0, 1];
    else
        feetInContact = [1, 1];
    end
    

    %% Update the time value
    if leftAsFixedLink
        
        if RFoot_wrench(3) > Config.threshold_contact_on && round(LFoot_is_fixed) > 0.9
            
            time = time + 0.01;
        else
            time = 0.0;
        end
    else
        if LFoot_wrench(3) > Config.threshold_contact_on && round(LFoot_is_fixed) < 0.1
            
            time = time + 0.01;
        else
            time = 0.0;
        end
    end
    
    %% Update leftAsFixedLink and reset w_H_fixedLink transform
    if leftAsFixedLink
     
        if time > 0.1
            
            time            = 0.0;
            leftAsFixedLink = false;
            
            % relative feet transform
            l_H_r           = b_H_l \ b_H_r;
            eulerAnglesL    = rollPitchYawFromRotation(l_H_r(1:3,1:3));
            yawAngleL       = eulerAnglesL(3);
            
            % reset the relative transform
            resetted_l_H_r          = zeros(4,4);
            resetted_l_H_r(1:3,1:3) = rotz(yawAngleL);
            resetted_l_H_r(4,4)     = 1;
            resetted_l_H_r(1:2,4)   = l_H_r(1:2,4);
            
            % reset the fixed link
            w_H_fixedLink           = w_H_fixedLink * resetted_l_H_r;
        end
    else
       
        if time > 0.1
            
            time            = 0.0;
            leftAsFixedLink = true;
            
            % relative feet transform
            r_H_l           = b_H_r \ b_H_l;
            eulerAnglesR    = rollPitchYawFromRotation(r_H_l(1:3,1:3));
            yawAngleR       = eulerAnglesR(3);
            
            % reset the relative transform
            resetted_r_H_l          = zeros(4,4);
            resetted_r_H_l(1:3,1:3) = rotz(yawAngleR);
            resetted_r_H_l(4,4)     = 1;
            resetted_r_H_l(1:2,4)   = r_H_l(1:2,4);
            
            % reset the fixed link
            w_H_fixedLink           = w_H_fixedLink * resetted_r_H_l;
        end
    end
    
    %% Base to world transform
    if leftAsFixedLink
        
        w_H_b                 = w_H_fixedLink/b_H_l;
        eulerAnglesIMU        = rollPitchYawFromRotation(w_R_b);
        eulerAnglesFK         = rollPitchYawFromRotation(w_H_b(1:3,1:3));        
        w_H_bWithImu(1:3,1:3) = rotz(eulerAnglesFK(3))*roty(eulerAnglesIMU(2))*rotx(eulerAnglesIMU(1));
        w_H_bWithImu(1:3,4)   = w_H_fixedLink(1:3,4) - w_H_bWithImu(1:3,1:3)*b_H_l(1:3,4);
    else
        w_H_b = w_H_fixedLink/b_H_r;
        eulerAnglesIMU        = rollPitchYawFromRotation(w_R_b);
        eulerAnglesFK         = rollPitchYawFromRotation(w_H_b(1:3,1:3));        
        w_H_bWithImu(1:3,1:3) = rotz(eulerAnglesFK(3))*roty(eulerAnglesIMU(2))*rotx(eulerAnglesIMU(1));
        w_H_bWithImu(1:3,4)   = w_H_fixedLink(1:3,4) - w_H_bWithImu(1:3,1:3)*b_H_r(1:3,4);
    end
    w_R_imu         = w_H_bWithImu(1:3,1:3) * b_R_imu;
    omega_world     = w_R_imu * [deg2rad(waistImu(7));deg2rad(waistImu(8));deg2rad(waistImu(9))];
    leftIsFixed_out = leftAsFixedLink;

    %% Update state and references
    state            = currentState;
       
    % Joint references
    s_des            =  s_des_walking;
    references_s     = [s_des, zeros(size(s_0,1),2)];
         
    % CoM references
    references_CoM   = [pos_vel_acc_CoM_des_walking(1:3), pos_vel_acc_CoM_des_walking(4:6), pos_vel_acc_CoM_des_walking(7:9)];

    references_LFoot = [w_walking_H_LFoot(1:3,4),   zeros(3,4);
                        w_walking_H_LFoot(1:3,1:3), zeros(3,2)];
    references_RFoot = [w_walking_H_RFoot(1:3,4),   zeros(3,4);
                        w_walking_H_RFoot(1:3,1:3), zeros(3,2)];
                                         
    %% Update rotational task reference
    
    % compute roll-pitch-yaw from rotation matrix for both feet
    roll_pitch_yaw_LFoot = rollPitchYawFromRotation(w_walking_H_LFoot(1:3,1:3));
    roll_pitch_yaw_RFoot = rollPitchYawFromRotation(w_walking_H_RFoot(1:3,1:3)); 
    
    % compute the mean of the two yaw angles
    mean_yaw = atan2(sin(roll_pitch_yaw_LFoot(3))+sin(roll_pitch_yaw_RFoot(3)), cos(roll_pitch_yaw_LFoot(3))+cos(roll_pitch_yaw_RFoot(3)));
     
    % ASSUMPTION: THE FRAME "w_walking" HAS THE SAME ORIENTATION OF FEET
    % FRAMES AT TIME 0. In this case, this is the relative rotation between 
    % the w_walking frame and a frame whose yaw is rotated by "mean_yaw" angle
    w_walking_R_w_yaw_rotated = rotz(-mean_yaw);
    
    % update rotational task references
    w_walking_R_rot_task = w_H_rot_task_initial(1:3,1:3);
    w_R_rot_task         = transpose(w_walking_R_w_yaw_rotated)*w_walking_R_rot_task;
    references_rot_task  = [w_R_rot_task, zeros(3,2)];
    
    tState = tState + 0.01;
end
