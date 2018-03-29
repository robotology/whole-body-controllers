% STATEMACHINEBALANCING computes robot state and references of the walking
%                     with MPC demo.
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT:  [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s] = stateMachineBalancing ...
%              (s_0, w_H_rot_task_0, w_H_CoM_0, w_H_LFoot_0, w_H_RFoot_0, b_H_LFoot, b_H_RFoot, LFoot_wrench, RFoot_wrench, Config, Sat)
%
% INPUT:  - s_0 = [ROBOT_DOF * 1] initial joints positions
%         - b_H_LFoot = [4 * 4] base to LFoot transform
%         - b_H_RFoot = [4 * 4] base to RFoot transform
%         - w_H_rot_task_0 = [4 * 4] world to rot task transform in the initial configuration
%         - w_H_CoM_0 = [4 * 4] world to CoM transform in the initial configuration
%         - w_H_LFoot_0 = [4 * 4] world to LFoot transform in the initial configuration
%         - w_H_RFoot_0 = [4 * 4] world to RFoot transform in the initial configuration
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
%         - references_rot_task = [3 * 5] desired rotational task link orientation, angular velocity and acceleration.
%                                 NOTE that the format is: [Rotation, angular_vel, angular_acc]     
%         - feetInContact = [2 * 1] feet in contact
%         - references_s = [ROBOT_DOF * 3] desired joint positions, velocities and accelerations
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function  [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s] = stateMachineBalancing ...
             (s_0, w_H_rot_task_0, w_H_CoM_0, w_H_LFoot_0, w_H_RFoot_0, b_H_LFoot, b_H_RFoot, LFoot_wrench, RFoot_wrench, Config, Sat)
              
    % State selector:
    %
    % state = 1 two feet blancing
    % state = 2 move CoM on left foot
    % state = 3 left foot balancing
    %

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% TODO: IMPROVE THE STATE MACHINE LOGIC (FOR NOW, IT IS USED FOR HOMING THE ROBOT 
    %% FROM TWO FEET TO ONE FOOT BALANCING
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    % Initialize variables
    persistent time
    
    if isempty(time)
        
        time = 0;       
    end
    
    state               = 1;  
    references_CoM      = [[w_H_CoM_0(1,4);w_H_CoM_0(2,4);w_H_CoM_0(3,4)], zeros(3,2)];
    references_LFoot    = [w_H_LFoot_0(1:3,4), zeros(3,4);
                           w_H_LFoot_0(1:3,1:3), zeros(3,2)];
    references_RFoot    = [w_H_RFoot_0(1:3,4), zeros(3,4);
                           w_H_RFoot_0(1:3,1:3), zeros(3,2)];
    references_rot_task = [w_H_rot_task_0(1:3,1:3), zeros(3,2)];
    feetInContact       = [1,1];
    references_s        = [s_0, zeros(size(s_0,1),2)];
  
    if time > 2
        
        state           = 2;
        references_CoM  = [[w_H_LFoot_0(1,4);w_H_LFoot_0(2,4);w_H_CoM_0(3,4)], zeros(3,2)];
        
        if RFoot_wrench(3) < Config.threshold_contact_off
            
            state = 3;
        end
    end
    
    if state == 3
        
        feetInContact       = [1,0];
        
        references_RFoot    = [w_H_RFoot_0(1:3,4)+0.02, zeros(3,4);
                               w_H_RFoot_0(1:3,1:3), zeros(3,2)];
              
    end
    
    time = time + 0.01;
end
