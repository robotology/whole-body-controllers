% STATEMACHINEEXAMPLE computes robot state and references of the EXAMPLE_STATEMACHINE demo.                   
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b] = stateMachineExample ...
%            (s_0, w_H_CoM_0, w_H_LFoot_0, w_H_RFoot_0, w_H_rot_task_0, t, LFoot_H_b, RFoot_H_b, LFoot_wrench, RFoot_wrench, Config)
%
% INPUT:  - s_0 = [ROBOT_DOF * 1] initial joint positions
%         - w_H_CoM_0 = [4 * 4] initial world to CoM transform
%         - w_H_LFoot_0 = [4 * 4] initial world to LFoot transform
%         - w_H_RFoot_0 = [4 * 4] initial world to RFoot transform
%         - w_H_rot_task_0 = [4 * 4] initial world to rotational task Link transform
%         - t = simulation time
%         - LFoot_H_b = [4 * 4] base to left foot transform
%         - RFoot_H_b = [4 * 4] base to right foot transform
%         - LFoot_wrench = [6 * 1] external forces and moments acting on the left foot
%         - RFoot_wrench = [6 * 1] external forces and moments acting on the right foot
%         - Config = user defined configuration
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
             (s_0, w_H_CoM_0, w_H_LFoot_0, w_H_RFoot_0, w_H_rot_task_0, t, LFoot_H_b, RFoot_H_b, LFoot_wrench, RFoot_wrench, Config)
          
     persistent currentState t_switch w_H_fixed_link
     
     if isempty(currentState) || isempty(t_switch) || isempty(w_H_fixed_link)
         
         currentState = 1;
         t_switch = 0;
         w_H_fixed_link = eye(4);
     end
     
     % STATE = 1: two feet balancing 
     %
     % Compute initial references for two feet balancing
     references_s        = [s_0, zeros(size(s_0,1),2)];
     references_CoM      = [w_H_CoM_0(1:3,4), zeros(3,2)];
     references_LFoot    = [w_H_LFoot_0(1:3,4), zeros(3,4);
                            w_H_LFoot_0(1:3,1:3), zeros(3,2)];
     references_RFoot    = [w_H_RFoot_0(1:3,4), zeros(3,4);
                            w_H_RFoot_0(1:3,1:3), zeros(3,2)];
     references_rot_task = [w_H_rot_task_0(1:3,1:3), zeros(3,2)];
    
     % Feet in contact
     feetInContact = [1,1];
    
     % World to base transform (world frame coincides with the fixed link at 0)
     if ~Config.LFoot_in_contact_at0
         
         w_H_b = w_H_fixed_link*RFoot_H_b;
     else
         w_H_b = w_H_fixed_link*LFoot_H_b;
     end
     
     % Change state
     if ~Config.ONLY_BALANCING && t > Config.t_balancing && currentState == 1
        
        currentState = 2;
        t_switch = t;
     end
     
              
     % STATE = 2: move CoM above the left foot
     %
     if currentState == 2
         
        references_CoM = [[w_H_LFoot_0(1:2,4);w_H_CoM_0(3,4)], zeros(3,2)];
       
        % Switch to the next state
        if t > (t_switch + Config.t_balancing) 
            
            currentState = 3; % left foot balancing
            t_switch = t;
            
            % The references must be updated if 
            % Config.LFoot_in_contact_at0 = false 
            if ~Config.LFoot_in_contact_at0 
               
                w_H_fixed_link = w_H_fixed_link*RFoot_H_b/LFoot_H_b;            
            end
        end
     end
        
     % STATE = 3: left foot balancing
     %
     if currentState == 3
     
         % World to base transform 
         w_H_b = w_H_fixed_link*LFoot_H_b;
     
         % Keep CoM on left foot    
         references_CoM = [[w_H_LFoot_0(1:2,4);w_H_CoM_0(3,4)], zeros(3,2)];
     
         % Move right foot backward and up
         references_RFoot    = [(w_H_RFoot_0(1:3,4)+transpose(Config.deltaPos_RFoot(currentState,:))), zeros(3,4);
                                 w_H_RFoot_0(1:3,1:3), zeros(3,2)];
    
         % Feet in contact
         feetInContact = [1,0];
     
         % Switch to the next state
         if t > (t_switch + Config.t_balancing) 
            
            currentState = 4; % prepare for switching
            t_switch = t;
         end 
     end
     
     % STATE = 4: prepare for switching
     %
     if currentState == 4
     
         % World to base transform 
         w_H_b = w_H_fixed_link*LFoot_H_b;
     
         % Keep CoM on left foot    
         references_CoM = [[w_H_LFoot_0(1:2,4);w_H_CoM_0(3,4)], zeros(3,2)];
    
         % Feet in contact
         feetInContact = [1,0];
     
         % Switch to the next state
         if t > (t_switch + Config.t_balancing) 
            
            currentState = 5; % back to two feet balancing
            t_switch = t;
         end 
     end
       
     % STATE = 5: back to two feet balancing
     %
     if currentState == 5
     
         % World to base transform 
         w_H_b = w_H_fixed_link*LFoot_H_b;
    
         % Feet in contact
         feetInContact = [1,1];
     end
     
     % update state
     state = currentState;
end