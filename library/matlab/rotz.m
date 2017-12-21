% ROTZ computes a rotation along the z axis (rotation matrix).
%
% FORMAT: R = rotz(alpha)     
%
% INPUT:  - alpha = angle in radians
%
% OUTPUT: - R = [3 * 3] rotation matrix along z axis
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function R = rotz(alpha)

   R      =  zeros(3, 3);
   R(3,3) =  1;
   R(1,1) =  cos(alpha);
   R(1,2) = -sin(alpha);
   R(2,1) =  sin(alpha);
   R(2,2) =  cos(alpha); 

end
