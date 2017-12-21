% FROMTRANSFMATRIXTOQUATERNIONS computes a link pose using position 
%                               + quaternion rapresentation. The input is 
%                               the link pose represented by a transformation matrix.
%
% FORMAT: q = fromTransfMatrixToPosQuat(H)  
%
% INPUT:  - H = [4 * 4] transf matrix representing link pose
%
% OUTPUT: - q = [7 * 1] position + quaternions representing link pose
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function q = fromTransfMatrixToPosQuat(H)

    % Separate the rotation matrix
    R = H(1:3,1:3);

    % Define the elements of the rotation matrix
    r00 = R(1,1);
    r11 = R(2,2);
    r22 = R(3,3);

    r21 = R(3,2);
    r12 = R(2,3);
    r02 = R(1,3);

    r20 = R(3,1);
    r10 = R(2,1);
    r01 = R(1,2);

    % Generate the quaternion
    qw = sqrt(1 +r00 +r11 +r22)/2;
    qx = (r21 -r12)/(4*qw);
    qy = (r02 -r20)/(4*qw);
    qz = (r10 -r01)/(4*qw);

    qt_b   = [qw; qx; qy; qz];

    % Final state converted into quaternion rapresentation
    q = [H(1:3,4); qt_b];
end
