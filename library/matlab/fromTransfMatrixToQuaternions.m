% FROMTRANSFMATRIXTOQUATERNIONS computes the robot state using position + 
%                               quaternion rapresentation for the floating 
%                               base pose. The input is the robot state with
%                               floating base pose represented by a 
%                               transformation matrix.
%
% FORMAT: q_quat = fromTransfMatrixToQuaternions(q_transf)  
%
% INPUT:  - q_transf = [16 +ROBOT_DOF * 1] robot state with transf matrix as base pose
%
% OUTPUT: - q_quat = [7 +ROBOT_DOF * 1] robot state with pos + quaternions as base pose
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function q_quat = fromTransfMatrixToQuaternions(q_transf)

    % Select the base pose from state vector
    pose_b   = q_transf(1:16);

    % Separate the rotation matrix
    w_R_b    = [q_transf(1:3) q_transf(5:7) q_transf(9:11)];

    % Define the elements of the rotation matrix
    r00 = w_R_b(1,1);
    r11 = w_R_b(2,2);
    r22 = w_R_b(3,3);

    r21 = w_R_b(3,2);
    r12 = w_R_b(2,3);
    r02 = w_R_b(1,3);

    r20 = w_R_b(3,1);
    r10 = w_R_b(2,1);
    r01 = w_R_b(1,2);

    % Generate the quaternion
    qw = sqrt(1 +r00 +r11 +r22)/2;
    qx = (r21 -r12)/(4*qw);
    qy = (r02 -r20)/(4*qw);
    qz = (r10 -r01)/(4*qw);

    qt_b   = [qw; qx; qy; qz];

    % Final state converted into quaternion rapresentation
    q_quat = [ pose_b(13:15); qt_b; q_transf(17:end)];
end
