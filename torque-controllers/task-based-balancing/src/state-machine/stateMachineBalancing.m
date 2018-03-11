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
function [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b, w_H_bWithImu, leftIsFixed_out, useFeetPIDs, omega_world] = stateMachineBalancing ...
             (s_0, w_H_LFoot_0, w_H_RFoot_0, w_H_rot_task_0, w_H_CoM_right, w_H_CoM_left, w_H_CoM_0, b_H_LFoot, b_H_RFoot, LFoot_wrench, RFoot_wrench, waistImu, Config, Sat)
         
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
    persistent w_R_wImu
   
    if isempty(currentState)
        
        currentState = 1;
    end
    if isempty(w_H_fixedLink)
       
        w_H_fixedLink = eye(4);
    end 
    if isempty(time)
       
        time = 0.0;
    end
    if isempty(leftAsFixedLink)
       
        leftAsFixedLink = true;
    end
    
    b_R_imu = rotz(-1.57079632679)*roty(0)*rotx(-2.09439521059);

    useFeetPIDs = true;
    
    if isempty(w_R_wImu)
        
        wImu_R_imu_0       = rotz(deg2rad(waistImu(3)))*roty(deg2rad(waistImu(2)))*rotx(deg2rad(waistImu(1)));
        w_H_b_0            = eye(4)/b_H_LFoot;
        w_R_b_0            = w_H_b_0(1:3,1:3);
        w_R_imu_0          = w_R_b_0*b_R_imu;
        w_R_wImu           = w_R_imu_0/(wImu_R_imu_0);
    end
    
    % base to imu rotation (fixed) and initial base to world rotation (fixed)
    wImu_R_imu   = rotz(deg2rad(waistImu(3)))*roty(deg2rad(waistImu(2)))*rotx(deg2rad(waistImu(1)));
    w_R_imu      = w_R_wImu*wImu_R_imu;
    omega_world  = w_R_imu * [deg2rad(waistImu(7));deg2rad(waistImu(8));deg2rad(waistImu(9))];
    w_R_b        = w_R_imu/b_R_imu;
    w_H_bWithImu = eye(4);
            
    %% State Selector
    if LFoot_wrench(3) > Config.threshold_contact_twoFeet && RFoot_wrench(3) > Config.threshold_contact_twoFeet && (currentState ~= 1) && (currentState == 2)
   
       currentState    = 1;
       leftAsFixedLink = true;
       time = 0.0;
       
    elseif LFoot_wrench(3) > Config.threshold_contact_twoFeet && RFoot_wrench(3) > Config.threshold_contact_twoFeet && (currentState ~= 1) && (currentState == 3)
       
        currentState    = 1;
        leftAsFixedLink = true;
        time = 0.0;
        
        % relative feet transform
        r_H_l           = b_H_RFoot \ b_H_LFoot;

        % reset the fixed link
        w_H_fixedLink           = w_H_fixedLink * r_H_l;
        w_H_fixedLink(3,4)      = 0.0;
        
    elseif  RFoot_wrench(3) < Config.threshold_contact_off && currentState ~= 2
        
        currentState    = 2;
        leftAsFixedLink = true;
       
    elseif  LFoot_wrench(3) < Config.threshold_contact_off && currentState ~= 3    
        
        currentState    = 3;        
        leftAsFixedLink = false;
        
        % relative feet transform
        l_H_r           = b_H_LFoot \ b_H_RFoot;
            
        % reset the fixed link
        w_H_fixedLink       = w_H_fixedLink * l_H_r;
        w_H_fixedLink(3,4)  = 0.0; 
    end
    
    %% Feet in contact selector    
    if currentState == 2
        
        feetInContact = [1, 0];
    
    elseif currentState == 3
       
        feetInContact = [0, 1];
    else
        feetInContact = [1, 1];
    end
    
    %% Update references
    if currentState == 1
            
    % CoM references
    references_CoM   = [w_H_CoM_0(1:3,4),zeros(3,2)];

    references_LFoot = [w_H_LFoot_0(1:3,4),   zeros(3,4);
                        w_H_LFoot_0(1:3,1:3), zeros(3,2)];
    references_RFoot = [w_H_RFoot_0(1:3,4),   zeros(3,4);
                        w_H_RFoot_0(1:3,1:3), zeros(3,2)];
                    
                    if time > 3
                        
                        references_CoM   = [[w_H_fixedLink(1:2,4);w_H_CoM_0(3,4)],zeros(3,2)];
                    end
                    
    elseif currentState == 2
        
    % CoM references
    references_CoM   = [[w_H_fixedLink(1:2,4);w_H_CoM_0(3,4)],zeros(3,2)];

    references_LFoot = [w_H_LFoot_0(1:3,4),   zeros(3,4);
                        w_H_LFoot_0(1:3,1:3), zeros(3,2)];
    references_RFoot = [w_H_RFoot_0(1:3,4),   zeros(3,4);
                        w_H_RFoot_0(1:3,1:3), zeros(3,2)];
                    
    elseif currentState == 3
        
    % CoM references
    references_CoM   = [[w_H_fixedLink(1:2,4);w_H_CoM_0(3,4)],zeros(3,2)];

    references_LFoot = [w_H_LFoot_0(1:3,4),   zeros(3,4);
                        w_H_LFoot_0(1:3,1:3), zeros(3,2)];
    references_RFoot = [w_H_RFoot_0(1:3,4),   zeros(3,4);
                        w_H_RFoot_0(1:3,1:3), zeros(3,2)];
       
    end
    
    %% Base to world transform
    if leftAsFixedLink
        
        w_H_b = w_H_fixedLink/b_H_l;
        eulerAnglesIMU = rollPitchYawFromRotation(w_R_b);
        eulerAnglesFK = rollPitchYawFromRotation(w_H_b(1:3,1:3));        
        w_H_bWithImu(1:3,1:3) = rotz(eulerAnglesFK(3))*roty(eulerAnglesIMU(2))*rotx(eulerAnglesIMU(1));
        w_H_bWithImu(1:3,4) = w_H_fixedLink(1:3,4) - w_H_bWithImu(1:3,1:3)*b_H_l(1:3,4);
    else
        w_H_b = w_H_fixedLink/b_H_r;
        eulerAnglesIMU = rollPitchYawFromRotation(w_R_b);
        eulerAnglesFK = rollPitchYawFromRotation(w_H_b(1:3,1:3));        
        w_H_bWithImu(1:3,1:3) = rotz(eulerAnglesFK(3))*roty(eulerAnglesIMU(2))*rotx(eulerAnglesIMU(1));
        w_H_bWithImu(1:3,4) = w_H_fixedLink(1:3,4) - w_H_bWithImu(1:3,1:3)*b_H_r(1:3,4);
    end
    
    leftIsFixed_out = leftAsFixedLink;

    %% Update state and references
    state            = currentState;
       
    % Joint  and rotational task references
    references_s         = [s_0, zeros(size(s_0,1),2)];
    references_rot_task  = [w_H_rot_task_0(1:3,1:3), zeros(3,2)];     
                                 
     % update time
    time = time + 0.01;
end
