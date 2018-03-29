% COMPOSETASKPOSEANDVELOCITY a function for composing task position and
%                            orientation, and task velocity.
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [pos_vel_CoM, pose_vel_LFoot, pose_vel_RFoot, orient_vel_rot_task] = ...
%             composeTaskPoseAndVelocity(vel_CoM, vel_LFoot, vel_RFoot, vel_rot_task, ...
%                                        w_H_CoM, w_H_l_sole, w_H_r_sole, w_H_rot_task)    
%
% INPUT: - vel_CoM = [3 * 1] CoM velocity
%        - vel_LFoot = [6 * 1] LFoot velocity
%        - vel_RFoot = [6 * 1] RFoot velocity
%        - vel_rot_task = [3 * 1] rotational task velocity
%        - w_H_CoM = [4 * 4] pose CoM
%        - w_H_l_sole = [4 * 4] pose LFoot
%        - w_H_r_sole = [4 * 4] pose RFoot
%        - w_H_rot_task = [4 * 4] pose rotational task
%
% OUTPUT: - pos_vel_CoM = [3 * 2] CoM position and velocity
%         - pose_vel_LFoot = [6 * 4] LFoot pose and velocity
%         - pose_vel_RFoot = [6 * 4] RFoot pose and velocity
%         - orient_vel_rot_task = [3 * 4] rotational task link orientation and angular velocity 
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function [pos_vel_CoM, pose_vel_LFoot, pose_vel_RFoot, orient_vel_rot_task] = ...
             composeTaskPoseAndVelocity(vel_CoM, vel_LFoot, vel_RFoot, vel_rot_task, ...
                                        w_H_CoM, w_H_l_sole, w_H_r_sole, w_H_rot_task)
                                                                    
    % compose CoM position and velocity
    pos_vel_CoM     = [w_H_CoM(1:3,4), vel_CoM];
    
    % compose LFoot pose and velocity
    pose_vel_LFoot  = [w_H_l_sole(1:3,4),    vel_LFoot(1:3), zeros(3,2);
                       w_H_l_sole(1:3,1:3),  vel_LFoot(4:6)];
                  
    % compose RFoot pose and velocity
    pose_vel_RFoot  = [w_H_r_sole(1:3,4),    vel_RFoot(1:3), zeros(3,2);
                       w_H_r_sole(1:3,1:3),  vel_RFoot(4:6)];
                  
    % compose Root orientation and velocity
    orient_vel_rot_task = [w_H_rot_task(1:3,1:3), vel_rot_task];
end