% ROTATIONDERIVATIVEFROMANGVEL computes the derivative of a rotation matrix 
%                              given the angular velocity.
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: RDot = rotationDerivativeFromAngVel(omega,R)
%
% INPUT:  - omega = [3 * 1] angular velocity
%         - R = [3 * 3] rotation matrix
%
%
% OUTPUT: - RDot = [3 * 3] rotation matrix derivative
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function RDot = rotationDerivativeFromAngVel(omega,R)

    % Rotation matrix derivative (with correction to keep the integration
    % inside the space of rotation matrices)
    kCorr   = 1;
    RDot    = R*skew(omega) +kCorr*(eye(3)-R*transpose(R))*R;
end