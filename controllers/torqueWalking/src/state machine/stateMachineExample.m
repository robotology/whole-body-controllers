% STATEMACHINEEXAMPLE computes robot state and references for the example
%                     state machine demo.
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
function [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b] = stateMachineExample ...
            (s_0, w_H_CoM_0, w_H_LFoot_0, w_H_RFoot_0, w_H_rot_task_0, fixed_link_H_b, LFoot_H_b, RFoot_H_b, t, Config)
        
 % Compute references
    references_s        = [s_0, zeros(size(s_0,1),2)];
    references_CoM      = [w_H_CoM_0(1:3,4), zeros(3,2)];
    references_LFoot    = [w_H_LFoot_0(1:3,4), zeros(3,4);
                           w_H_LFoot_0(1:3,1:3), zeros(3,2)];
    references_RFoot    = [w_H_RFoot_0(1:3,4), zeros(3,4);
                           w_H_RFoot_0(1:3,1:3), zeros(3,2)];
    references_rot_task = [w_H_rot_task_0(1:3,1:3), zeros(3,2)];
    
    % Current state
    state = 1;
    
    % Feet in contact
    feetInContact = [1,1];
    
    % World to base transform (world frame coincides with the fixed link)
    w_H_b = fixed_link_H_b;
    
    % Move CoM on left foot after 5 seconds
    if t > 5
        
        state = 2;
        references_CoM = [[w_H_RFoot_0(1:2,4);w_H_CoM_0(3,4)], zeros(3,2)];
    end
    
    % Left foot balancing
    if t > 10
        
        state = 3;
        feetInContact = [0,1];
        references_LFoot = [[w_H_LFoot_0(1:2,4);w_H_LFoot_0(3,4)+0.02], zeros(3,4);
                             w_H_LFoot_0(1:3,1:3), zeros(3,2)];
    end
    
    % Move the foot back
    if t > 15

        references_LFoot = [[w_H_LFoot_0(1,4)-0.05;w_H_LFoot_0(2,4);w_H_LFoot_0(3,4)+0.05], zeros(3,4);
                             w_H_LFoot_0(1:3,1:3), zeros(3,2)];
    end
               
end