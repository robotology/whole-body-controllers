% GAINSCHEDULING given the current state, it assigns feedback gains for the
%                joint and tasks dynamics.
%
% FORMAT: [Kp_CoM, Kd_CoM, Kp_LFoot, Kd_LFoot, Kp_RFoot, Kd_RFoot, ...
%          Kp_Rot_task, Kd_Rot_task, impedances, dampings, smoothingTimeGains] = ...
%          gainScheduling(state,Gains,Config);   
%
% INPUT:  - state = current robot state
%         - Gains = structure containing the tasks and joints gains
%         - Config = structure of user-defined configuration parameters
%
% OUTPUT: - Kp_CoM = [3 * 1] CoM position gains
%         - Kd_CoM = [3 * 1] CoM velocity gains
%         - Kp_LFoot = [6 * 1] LFoot position gains
%         - Kd_LFoot = [6 * 1] LFoot velocity gains
%         - Kp_RFoot = [6 * 1] RFoot position gains
%         - Kd_RFoot = [6 * 1] RFoot velocity gains
%         - Kp_rot_task = [3 * 1] rot task position gains
%         - Kd_rot_task = [3 * 1] rot task velocity gains
%         - impedances = [ROBOT_DOF * 1] joints position gains
%         - dampings = [ROBOT_DOF * 1] joints velocity gains
%         - smoothingTimeGains = smoothing time for min jerk trajectory generator
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function [Kp_CoM, Kd_CoM, Kp_LFoot, Kd_LFoot, Kp_RFoot, Kd_RFoot, Kp_rot_task, Kd_rot_task, impedances, dampings, smoothingTimeGains] = ...
          gainScheduling(state,Gains,Config)
      
    % Select the gains according to the current state
    Kp_CoM      = transpose(Gains.Kp_CoM(state,:));
    Kd_CoM      = transpose(Gains.Kd_CoM(state,:)); 
    Kp_LFoot    = transpose(Gains.Kp_LFoot(state,:));
    Kd_LFoot    = transpose(Gains.Kd_LFoot(state,:)); 
    Kp_RFoot    = transpose(Gains.Kp_RFoot(state,:)); 
    Kd_RFoot    = transpose(Gains.Kd_RFoot(state,:)); 
    Kp_rot_task = transpose(Gains.Kp_rot_task(state,:)); 
    Kd_rot_task = transpose(Gains.Kd_rot_task(state,:));  
    impedances  = transpose(Gains.impedances(state,:));
    dampings    = transpose(Gains.dampings(state,:));
    
    % Select the current smoothing time
    smoothingTimeGains = Config.smoothingTimeGains(state);
end