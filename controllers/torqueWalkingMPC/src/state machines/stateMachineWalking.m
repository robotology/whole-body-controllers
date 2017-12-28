% STATEMACHINEWALKING computes robot state and references of the walking
%                     with MPC demo.
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b] = stateMachineWalking ...
%             (s_0, pos_vel_acc_CoM_des_walking, s_des_walking, feetInContact_walking, LFoot_H_b, RFoot_H_b, w_H_rot_task_0, w_walking_H_b)
%
% INPUT:  - s_0 = [ROBOT_DOF * 1] initial joints positions
%         - pos_vel_acc_CoM_des_walking = [3 * 3] CoM references from MPC
%         - s_des_walking = [ROBOT_DOF * 1] joints references from MPC
%         - feetInContact_walking  = [2 * 1] active contacts with MPC
%         - LFoot_H_b = [4 * 4] base to  LFoot transform
%         - RFoot_H_b = [4 * 4] base to  RFoot transform
%         - w_H_rot_task_0 = [4 * 4] rot task to world transform
%         - w_walking_H_b = [4 * 4] base to world (MPC) transform
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
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b] = stateMachineWalking ...
             (s_0, pos_vel_acc_CoM_des_walking, s_des_walking, feetInContact_walking, w_walking_H_LFoot, w_walking_H_RFoot, w_H_rot_task_0, w_walking_H_b)
         
    % State selector:
    %
    % state = 1 two feet blancing
    % state = 2 left foot balancing
    % state = 3 right foot balancing
    %
    toll  = 0.1;
    state = 1;
    
    if feetInContact_walking(2) < toll
        
        state = 2;
        
    elseif feetInContact_walking(1) < toll
        
        state = 3;
    end
    
    % Base to world transform
    w_H_b = w_walking_H_b;
    
    % Active contacts
    feetInContact = transpose(feetInContact_walking);
       
    % Joint references
    s_des        = [s_des_walking(1:3); s_0(4:11); s_des_walking(4:15)];
    references_s = [s_des, zeros(size(s_0,1),2)];
         
    % CoM references
    references_CoM = [pos_vel_acc_CoM_des_walking(1:3), pos_vel_acc_CoM_des_walking(4:6), pos_vel_acc_CoM_des_walking(7:9)];
    
    references_LFoot    = [w_walking_H_LFoot(1:3,4), zeros(3,4);
                           w_walking_H_LFoot(1:3,1:3), zeros(3,2)];
    references_RFoot    = [w_walking_H_RFoot(1:3,4), zeros(3,4);
                           w_walking_H_RFoot(1:3,1:3), zeros(3,2)];
                       
    %% Update rotational task reference.
    
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
    w_walking_R_rot_task = w_H_rot_task_0(1:3,1:3);
    w_R_rot_task         = transpose(w_walking_R_w_yaw_rotated)*w_walking_R_rot_task;
    references_rot_task  = [w_R_rot_task, zeros(3,2)];
    
end
