% INTERNALCOORDINATOR computes robot state and references of the INTERNAL
%                     COORDINATOR demo.                   
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b] = ...
%              internalCoordinator(s_0, w_H_CoM_0, w_H_LFoot_0, w_H_RFoot_0, w_H_rot_task_0, fixed_link_H_b, t, Config)
%
% INPUT:  - s_0 = [ROBOT_DOF * 1] initial joint positions
%         - w_H_CoM_0 = [4 * 4] initial world to CoM transform
%         - w_H_LFoot_0 = [4 * 4] initial world to LFoot transform
%         - w_H_RFoot_0 = [4 * 4] initial world to RFoot transform
%         - w_H_rot_task_0 = [4 * 4] initial world to rotational task Link transform
%         - fixed_link_H_b = [4 * 4] base to fixed link transform
%         - t = simulation time
%         - Config = user defined configuration
%
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
function [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b] = internalCoordinator ...
             (s_0, w_H_CoM_0, w_H_LFoot_0, w_H_RFoot_0, w_H_rot_task_0, fixed_link_H_b, t, Config)
         
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
    
    % Modify CoM references if DEMO MOVEMENTS is selected 
    pos_CoM_0 = w_H_CoM_0(1:3,4);
    
    if Config.DEMO_MOVEMENTS
        
        references_CoM = referenceGeneratorCoM(pos_CoM_0, t, Config);
    end     
end