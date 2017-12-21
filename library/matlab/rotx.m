% ROTX computes a rotation along the x axis (rotation matrix).
%
% FORMAT: R = rotz(alpha)     
%
% INPUT:  - alpha = angle in radians
%
% OUTPUT: - R = [3 * 3] rotation matrix along x axis
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function R = rotx(alpha)

   R      =  zeros(3, 3);
   R(1,1) =  1;
   R(2,2) =  cos(alpha);
   R(2,3) = -sin(alpha);
   R(3,2) =  sin(alpha);
   R(3,3) =  cos(alpha); 
   
end
