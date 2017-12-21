% ROTATIONFROMQUATERNION computes the rotation matrix from a quaternion by
%                        applying Rodrigues's formula:
%
%                            R = I_3 + 2*s*skew(r) + 2*skew(r)^2
%
% FORMAT: R = rotationFromQuaternion(q)  
%
% INPUT:  - q = [4 * 1] quaternion
%
% OUTPUT: - R = [3 * 3] rotation matrix
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function R = rotationFromQuaternion(q)

    % rotation matrix:
    R = eye(3) + 2*q(1)*skew(q(2:4)) + 2*skew(q(2:4))^2;
end
