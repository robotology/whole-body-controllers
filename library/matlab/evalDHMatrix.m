% EVALDHMATRIX calculates the DH matrix from angles and parameters.
%
% FORMAT: H = evalDHMatrix(a, d, alpha, theta)   
%
% INPUT:  - a = dimension (m)
%         - d = dimension (m)
%         - alpha = angle (rad)
%         - theta = angle (rad)
%
% OUTPUT: - H = [4 * 4] DH matrix
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function H = evalDHMatrix(a, d, alpha, theta)

    H = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), cos(theta)*a
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), sin(theta)*a
                   0,             sin(alpha),             cos(alpha),            d
                   0,                      0,                      0,            1];
                
end