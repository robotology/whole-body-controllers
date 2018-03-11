% ROTATIONALPID_ACCELERATION implements a controller trajectory tracking inside
%                            the group SO(3). The angular acceleration is assumed
%                            to be a control input.
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
function w_omegaDot_star = rotationalPID_acceleration(w_R_b,w_omega,w_R_b_des,w_omega_des,w_omegaDot_des,Kp,Kd)
    
    % Modified gains to use the control with the real robot
    c0 = 0.001;
    c1 = Kd;
    c2 = Kp;
    
    % ROTATIONAL PID (see also: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.62.8655&rep=rep1&type=pdf, section 5.11.6, p.173) 
    skv             = skewVee(w_R_b*transpose(w_R_b_des));
    skvDot          = skewVee(skew(w_omega)*w_R_b*transpose(w_R_b_des)-w_R_b*transpose(w_R_b_des)*skew(w_omega_des));
    omegaE_Dot      = w_omegaDot_des -c0*skvDot;
    
    w_omegaDot_star = omegaE_Dot -c1*(w_omega-w_omega_des) -c2*skv;
    
end
