% ADDMOTORSINERTIA adds the motors reflected inertias to the joint space
%                  mass matrix.
%
% PROCEDURE: the assume the floating base + joint + motors dynamics can be
% described by the following system of equations:
%
%     M_b*nuDot_b + M_bj*qjDDot + h_b = J_b^T*f (1)
%
%     M_j*qjDDot + M_jb*nuDot_b + h_j = J_j^T*f + tau_j (2)
%
%     I_m*thetaDDot = tau_m - transpose(T*Gamma)*tau_j (3)
%
% Assuming that the joint is rigidly attached to the motor, one has that:
%
%     thetaDDot = (T*Gamma)^-1*qjDDot (4)
%
% where Gamma is a diagonal matrix containing the transmission ratio for
% all joints, while T is a block diagonal matrix taking into account the 
% fact that some joint movements are obtained by combining the effect of 
% different motors (see also http://wiki.icub.org/wiki/ICub_coupled_joints).
% By substituting (4) into (3), multiplying (3) by transpose(T*Gamma)^-1 and
% finally summing up (3) and (2), one has:
%
%     M_b*nuDot_b + M_bj*qjDDot + h_b = J_b^T*f (1)
%
%    (M_j+transpose(T*Gamma)^-1*I_m*(T*Gamma)^-1)*qjDDot + M_jb*nuDot_b + h_j = J_j^T*f + u (2b)
%
% where u = transpose(T*Gamma)^-1*tau_m. The input joint toruqes can be calculated from
% (3) as follows:
%
%    tau_j = u - K_ff*transpose(T*Gamma)^-1*I_m*(T*Gamma)^-1*qjDDot 
%
% with K_ff belonging to [0,1].
%
% FORMAT: M_with_inertia = addMotorsInertia(M,Config)
%
% INPUT: -M = [6+n x 6+n] mass matrix
%        -Config = user defined configuration parameters
%
% OUTPUT: -M_with_inertia = [6+n x 6+n] mass matrix with motor reflected
%                           inertia
%
% Authors: Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function M_with_inertia = addMotorsInertia(M,Config)
    
     % parameters
     nDof                    = size(M,1)-6;
      
     % add motors reflected inertias
     if Config.USE_MOTOR_REFLECTED_INERTIA
         
         reflectedInertia    = computeMotorsInertia(Config);
         M_reflected_inertia = [zeros(6,6+nDof);
                                zeros(nDof,6) reflectedInertia];
     else
         
         M_reflected_inertia = zeros(size(M));
     end
         
     M_with_inertia = M + M_reflected_inertia;
end