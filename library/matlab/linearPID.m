% LINEARPID implements a PID controller for a Cartesian position task. The
%           task position belongs to the Euclidean space R^3.
%
% FORMAT: acc_star = linearPID(pos,vel,pos_des,vel_des,acc_des,Kp,Kd)     
%
% INPUT:  - pos = [3 * 1] task position
%         - vel = [3 * 1] task velocity
%         - pos_des = [3 * 1] task desired position
%         - vel_des = [3 * 1] task desired velocity
%         - acc_des = [3 * 1] task desired acceleration
%         - Kp = [3 * 3] position gains 
%         - Kd = [3 * 3] velocity gains
%
% OUTPUT: - acc_star = [3 * 1] desired task dynamics
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function acc_star = linearPID(pos,vel,pos_des,vel_des,acc_des,Kp,Kd) 
    
    % classical PID controller
    acc_star = acc_des -Kp*(pos-pos_des) -Kd*(vel-vel_des);    
end
