% POSROTTOTRANSFMATR computes the transformation matrix given as input a
%                    position vector and a rotation matrix
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: H = posRotToTransfMatrix(pos, R)
%
% INPUT:  - pos = [3 * 1] position vector
%         - R = [3 * 3] rotation matrix
%
% OUTPUT: - H = [4 * 4] transformation matrix
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Feb 2018
%

%% --- Initialization ---
function H = posRotToTransfMatrix(pos,R) 

    H           = eye(4);
    H(1:3,1:3)  = R;
    H(1:3, 4)   = pos; 
end