% ROTY computes a rotation along the y axis (rotation matrix).
%
% FORMAT: R = rotz(alpha)     
%
% INPUT:  - alpha = angle in radians
%
% OUTPUT: - R = [3 * 3] rotation matrix along y axis
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function R = roty(alpha)

   R      =  zeros(3, 3);
   R(2,2) =  1;
   R(1,1) =  cos(alpha);
   R(1,3) =  sin(alpha);
   R(3,1) = -sin(alpha);
   R(3,3) =  cos(alpha); 
   
end
