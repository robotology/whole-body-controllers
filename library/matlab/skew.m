% SKEW function to generate a [3 * 3] Skew Symmetric matrix out of a 
%      [3 * 1] vector.
%
% FORMAT: X = skew(x)  
%
% INPUT:  - x = [3 * 1] vector
%
% OUTPUT: - X = [3 * 3] skew symmetric matrix
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function X = skew(x)

    X=[ 0    -x(3)  x(2); 
        x(3)  0    -x(1); 
       -x(2)  x(1)  0 ];
end

