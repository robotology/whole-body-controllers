% ROTATIONALPID_VELOCITY implements a controller trajectory tracking inside
%                        the group SO(3). The angular velocity is assumed
%                        to be a control input.
%
% FORMAT: w_omega = rotationalPID_velocity(w_R_b,w_R_b_des,w_omega_des,Kp)    
%
% INPUT: - w_R_b       = [3 * 3] rotation matrix
%        - w_R_b_des   = [3 * 3] desired rotation matrix
%        - w_omega     = [3 * 1] angular velocity (expressed in the world frame)
%        - w_omega_des = [3 * 1] desired angular velocity (expressed in the world frame)
%        - Kp          = [3 * 3] orientation gains
%
% OUTPUT: - w_omega    = [3 * 1] input angular velocity
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function w_omega = rotationalPID_velocity(w_R_b,w_R_b_des,w_omega_des,Kp) 

    skv      = skewVee(w_R_b*transpose(w_R_b_des));
    w_omega  = w_omega_des -Kp*skv;
end