% COMPUTEBASEVELOCITY computes the floating base velocity assuming
%                     left/right foot is in contact with the ground.
%
% FORMAT: nu_b = computeBaseVelocity(J_LeftFoot,J_RightFoot, feetInContact, sDot, Sat)    
%
% INPUT:  - J_LeftFoot = [6 * ROBOT_DOF +6] left foot Jacobian
%         - J_RightFoot = [6 * ROBOT_DOF +6] right foot Jacobian
%         - feetInContact = [2 * 1] feet in contact
%         - sDot = [ROBOT_DOF * 1] joint velocity
%         - Sat = saturation parameters
%
% OUTPUT: - nu_b = [6 * 1] floating base velocity
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function nu_b = computeBaseVelocity(J_LeftFoot,J_RightFoot, feetInContact, sDot, Sat)

    % Compute Jacobian
    Jc = [feetInContact(1)*J_LeftFoot;
          feetInContact(2)*J_RightFoot];

    % Compute multiplier of nu_b  
    pinvJb = (transpose(Jc(:,1:6))*Jc(:,1:6) + Sat.pinvDamp_nu_b*eye(6))\transpose(Jc(:,1:6));  
  
    % Base velocity
    nu_b = -pinvJb*Jc(:,7:end)*sDot;
end