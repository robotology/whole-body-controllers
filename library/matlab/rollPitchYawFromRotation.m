% ROLLPITCHYAWFROMROTATION converts a rotation matrix into Euler angles
%                          (roll-pitch-yaw convention).
%
% FORMAT: rollPitchYaw = rollPitchYawFromRotation(R)     
%
% INPUT:  - R = [3 * 3] rotation matrix
%
% OUTPUT: - rollPitchYaw = [3 * 1] vector of Euler angles [rad]
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function rollPitchYaw = rollPitchYawFromRotation(R)

    % For documentation, see also:
    %
    % http://www.geometrictools.com/Documentation/EulerAngles.pdf
    %
    rollPitchYaw = zeros(3,1);

    if (R(3,1) < +1)
    
        if (R(3,1) > -1) 
            rollPitchYaw(2) = asin(-R(3,1)); 
            rollPitchYaw(3) = atan2(R(2,1),R(1,1)); 
            rollPitchYaw(1) = atan2(R(3,2), R(3,3));
        else
            rollPitchYaw(3) =-atan2(-R(2,3),R(2,2));
            rollPitchYaw(1) = 0;
        end
    else
        rollPitchYaw(3) = atan2(-R(2,3),R(2,2));
        rollPitchYaw(1) = 0;
    end
end