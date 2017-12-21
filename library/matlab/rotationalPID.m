% ROTATIONALPID implements a PID controller for an orientation task. The
%               task orientation belongs to the Special Orthogonal group SO(3).
%
% IMPLEMENTATION: a task orientation may be expressed through a [3 * 3] rotation 
%                 matrix w_R_b, which transforms a vector expressed w.r.t.
%                 a body frame b into a vector expressed w.r.t. the
%                 inertial frame w:
%
%                     p_w = w_R_b * p_b.
%
%                 The angular velocity of the tasks may be expressed as a 
%                 [3 * 1] vector such that:
%                   
%                     dot(w_R_b) = Skew(omega_w) * w_R_b.
%
%                 The objective of this function is to ensure the
%                 convergence of both w_R_b and omega_w to a desired task
%                 orientation and angular velocity. This is done by properly 
%                 computing the task angular accelerations (omegaDot_w). 
%
% FORMAT: w_omegaDot_star = rotationalPID(w_R_b,w_omega,w_R_b_des,
%                                         w_omega_des,w_omegaDot_des,Kp,Kd)    
%
% INPUT: - w_R_b = [3 * 3] rotation matrix
%        - w_omega = [3 * 1] angular velocity
%        - w_R_b_des = [3 * 3] desired rotation matrix
%        - w_omega_des = [3 * 1] desired angular velocity
%        - w_omegaDot_des = [3 * 1] desired angular accelerations
%        - Kp = [3 * 3] orientation gains
%        - Kd = [3 * 3] angular velocity gains
%
% OUTPUT: - omegaDot_star = [3 * 1] desired task angular accelerations
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function w_omegaDot_star = rotationalPID(w_R_b,w_omega,w_R_b_des,w_omega_des,w_omegaDot_des,Kp,Kd)
    
    % express the angular velocity w.r.t. the body frame
    b_omega      = transpose(w_R_b) * w_omega;
    b_omega_des  = transpose(w_R_b_des) * w_omega_des;
    
    % omegaDot_star in the body frame (rotational PID)
    b_omegaDot_star = -Kp*Kd*skewVee(transpose(w_R_b_des)*w_R_b)  ...
                      -Kd*(b_omega -b_omega_des)...
                      -Kp*skewVee(transpose(w_R_b_des)*w_R_b*skew(b_omega) -skew(b_omega_des)*transpose(w_R_b_des)*w_R_b);
    
    % omegaDot_star in the world frame
    w_omegaDot_star = w_R_b*b_omegaDot_star + w_omegaDot_des;
end
