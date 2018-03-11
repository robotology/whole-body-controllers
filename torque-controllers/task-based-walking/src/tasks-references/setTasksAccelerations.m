% SETTASKSACCELERATIONS this function sets the accelerations of some tasks 
%                       in a Cartesian space. For the time being, all the
%                       Cartesian tasks have the same priority.
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: acc_task_star = setTasksAccelerations(pos_vel_acc_CoM_des,pose_vel_acc_LFoot_des, ...
%                                               pose_vel_acc_RFoot_des,orient_vel_acc_rot_task_des, ...
%                                               pos_vel_CoM, pose_vel_LFoot, pose_vel_RFoot, orient_vel_rot_task, ...
%                                               Kp_Kd_CoM, Kp_Kd_LFoot, Kp_Kd_RFoot, Kp_Kd_rot_task, feetInContact, useFeetPIDs, Sat, Config)     
%
% INPUT: - pos_vel_acc_CoM_des = [3 * 3] desired CoM position, velocity and acceleration
%        - pose_vel_acc_LFoot_des = [6 * 5] desired LFoot pose, velocity
%                                   and acceleration. NOTE that the format
%                                   is: [pos, linear_vel, linear_acc, zeros(3,2);
%                                        Rotation, angular_vel, angular_acc] 
%        - pose_vel_acc_RFoot_des = [6 * 5] desired RFoot pose, velocity and acceleration
%        - orient_vel_acc_rot_task_des = [3 * 5] desired Rotational task link orientation, angular velocity and acceleration.
%                                        NOTE that the format is: [Rotation, angular_vel, angular_acc]                                     
%        - pos_vel_CoM = [3 * 2] CoM position and velocity
%        - pose_vel_LFoot = [6 * 4] LFoot pose and velocity. NOTE that the format
%                           is: [pos, linear_vel, zeros(3,1);
%                                Rotation, angular_vel]
%        - pose_vel_RFoot = [6 * 4] RFoot pose and velocity
%        - orient_vel_rot_task = [3 * 4] Rotational task link orientation and angular velocity 
%        - Kp_Kd_CoM = [3 * 2] CoM position and velocity gains
%        - Kp_Kd_LFoot = [6 * 2] LFoot pose and velocity gains
%        - Kp_Kd_RFoot = [6 * 2] RFoot pose and velocity gains
%        - Kp_Kd_rot_task = [3 * 2] Rotational task link orientation and angular velocity gains
%        - feetInContact = [2 * 1] feet contact status (0 not active, 1 active)
%        - useFeetPIDs = [2 * 1] boolean to activate PIDs at feet
%        - Sat = a structure containing user defined saturation parameters
%        - Config = user defined configuration
%
% OUTPUT: - acc_task_des = [12 * 1] vector of cartesian tasks accelerations
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function acc_task_star = setTasksAccelerations(pos_vel_acc_CoM_des,pose_vel_acc_LFoot_des,pose_vel_acc_RFoot_des,orient_vel_acc_rot_task_des, ...
                                               pos_vel_CoM, pose_vel_LFoot, pose_vel_RFoot, orient_vel_rot_task, ...
                                               Kp_Kd_CoM, Kp_Kd_LFoot, Kp_Kd_RFoot, Kp_Kd_rot_task, feetInContact, useFeetPIDs, Sat, Config)

    % LFoot position, and linear velocity
    pos_LFoot    = pose_vel_LFoot(1:3,1);
    vel_LFoot    = pose_vel_LFoot(1:3,2);
    
    % LFoot orientation, and angular velocity
    w_R_LFoot    = pose_vel_LFoot(4:6,1:3);
    omega_LFoot  = pose_vel_LFoot(4:6,4);
    
    % RFoot position, and linear velocity
    pos_RFoot    = pose_vel_RFoot(1:3,1);
    vel_RFoot    = pose_vel_RFoot(1:3,2);
    
    % RFoot orientation, and angular velocity
    w_R_RFoot    = pose_vel_RFoot(4:6,1:3);
    omega_RFoot  = pose_vel_RFoot(4:6,4);
    
    % CoM position, and linear velocity
    pos_CoM      = pos_vel_CoM(1:3,1);
    vel_CoM      = pos_vel_CoM(1:3,2);
    
    % Rotational task link orientation, and angular velocity
    w_R_rot_task     = orient_vel_rot_task(1:3,1:3);
    omega_rot_task   = orient_vel_rot_task(1:3,4);
        
    % LFoot desired position, linear velocity and acceleration
    pos_LFoot_des      = pose_vel_acc_LFoot_des(1:3,1);
    vel_LFoot_des      = pose_vel_acc_LFoot_des(1:3,2);
    acc_LFoot_des      = pose_vel_acc_LFoot_des(1:3,3);
    
    % LFoot desired orientation, angular velocity and acceleration
    w_R_LFoot_des      = pose_vel_acc_LFoot_des(4:6,1:3);
    omega_LFoot_des    = pose_vel_acc_LFoot_des(4:6,4);
    omegaDot_LFoot_des = pose_vel_acc_LFoot_des(4:6,5);
    
    % RFoot desired position, linear velocity and acceleration
    pos_RFoot_des      = pose_vel_acc_RFoot_des(1:3,1);
    vel_RFoot_des      = pose_vel_acc_RFoot_des(1:3,2);
    acc_RFoot_des      = pose_vel_acc_RFoot_des(1:3,3);
    
    % RFoot desired orientation, angular velocity and acceleration
    w_R_RFoot_des      = pose_vel_acc_RFoot_des(4:6,1:3);
    omega_RFoot_des    = pose_vel_acc_RFoot_des(4:6,4);
    omegaDot_RFoot_des = pose_vel_acc_RFoot_des(4:6,5);
    
    % CoM desired position, linear velocity and acceleration
    pos_CoM_des        = pos_vel_acc_CoM_des(1:3,1);
    vel_CoM_des        = pos_vel_acc_CoM_des(1:3,2);
    acc_CoM_des        = pos_vel_acc_CoM_des(1:3,3);
    
    % Rotational task link desired orientation, angular velocity and acceleration
    w_R_rot_task_des       = orient_vel_acc_rot_task_des(1:3,1:3);
    omega_rot_task_des     = orient_vel_acc_rot_task_des(1:3,4);
    omegaDot_rot_task_des  = orient_vel_acc_rot_task_des(1:3,5);
    
    % LFoot position and linear velocity gains
    Kp_LFoot       = diag(Kp_Kd_LFoot(1:3,1));
    Kd_LFoot       = diag(Kp_Kd_LFoot(1:3,2));
    
    % LFoot orientation and angular velocity gains
    Kp_rot_LFoot   = diag(Kp_Kd_LFoot(4:6,1));
    Kd_omega_LFoot = diag(Kp_Kd_LFoot(4:6,2));
    
    % RFoot position and linear velocity gains
    Kp_RFoot       = diag(Kp_Kd_RFoot(1:3,1));
    Kd_RFoot       = diag(Kp_Kd_RFoot(1:3,2));
    
    % RFoot orientation and angular velocity gains
    Kp_rot_RFoot   = diag(Kp_Kd_RFoot(4:6,1));
    Kd_omega_RFoot = diag(Kp_Kd_RFoot(4:6,2));
    
    % CoM position and linear velocity gains
    Kp_CoM         = diag(Kp_Kd_CoM(1:3,1));
    Kd_CoM         = diag(Kp_Kd_CoM(1:3,2));    
    
    % Rotational task link orientation and angular velocity gains
    Kp_rot_task    = diag(Kp_Kd_rot_task(1:3,1));
    Kd_rot_task    = diag(Kp_Kd_rot_task(1:3,2));
    
    %% Linear PID controllers for position tasks
    
    % desired left foot acceleration. If balancing on left foot, then
    % the desired left foot acceleration is zero
    if useFeetPIDs(1) > (1-Sat.toll_useRotPID)
        
        acc_LFoot_star = linearPID(pos_LFoot,vel_LFoot,pos_LFoot_des,vel_LFoot_des,acc_LFoot_des,Kp_LFoot,Kd_LFoot);     
    else
        acc_LFoot_star = zeros(3,1);
    end
        
    % desired right foot acceleration. If balancing on right foot, then
    % the desired right foot acceleration is zero
    if useFeetPIDs(2) > (1-Sat.toll_useRotPID) 
        
        acc_RFoot_star = linearPID(pos_RFoot,vel_RFoot,pos_RFoot_des,vel_RFoot_des,acc_RFoot_des,Kp_RFoot,Kd_RFoot);
    else
        acc_RFoot_star = zeros(3,1);
    end
    
    % desired CoM dynamics
    acc_CoM_star = linearPID(pos_CoM,vel_CoM,pos_CoM_des,vel_CoM_des,acc_CoM_des,Kp_CoM,Kd_CoM);
    
    %% Rotational PID controllers for orientation tasks

    % desired left foot angular acceleration. If balancing on left foot, 
    % then the desired left foot angular acceleration is zero
    if useFeetPIDs(1) > (1-Sat.toll_useRotPID)
        
        omegaDot_LFoot_star = rotationalPID_acceleration(w_R_LFoot, omega_LFoot, w_R_LFoot_des, omega_LFoot_des, omegaDot_LFoot_des, Kp_rot_LFoot, Kd_omega_LFoot);  
    else
        omegaDot_LFoot_star = zeros(3,1);
    end
        
    % desired right foot angular acceleration. If balancing on right foot, 
    % then the desired right foot angular acceleration is zero
    if useFeetPIDs(2) > (1-Sat.toll_useRotPID) 
        
        omegaDot_RFoot_star = rotationalPID_acceleration(w_R_RFoot, omega_RFoot, w_R_RFoot_des, omega_RFoot_des, omegaDot_RFoot_des, Kp_rot_RFoot, Kd_omega_RFoot); 
    else
        omegaDot_RFoot_star = zeros(3,1);
    end
    
    % desired rotational task link angular acceleration
    omegaDot_rot_task_star = rotationalPID_acceleration(w_R_rot_task, omega_rot_task, w_R_rot_task_des, omega_rot_task_des, omegaDot_rot_task_des, Kp_rot_task, Kd_rot_task);
    
    %% Tasks accelerations 
    
    % compose tasks accelerations. The task ordering is the following:
    %
    % - Left foot linear and angular acceleration
    % - Right foot linear and angular acceleration
    % - CoM linear acceleration
    % - Rotational task link angular acceleration
    %
    acc_task_star = [acc_LFoot_star; omegaDot_LFoot_star; ...
                     acc_RFoot_star; omegaDot_RFoot_star; ...
                     acc_CoM_star;   omegaDot_rot_task_star];
    
end
