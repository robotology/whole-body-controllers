function rollPitchYaw = rollPitchYawFromRotation(R)

    % ROLLPITCHYAWFROMROTATION converts a rotation matrix into Euler angles.
    %                          The Euler angles convention follows the one
    %                          of iDyntree and is such that the rotation
    %                          matrix is:  R = Rz(yaw)*Ry(pitch)*Rx(roll).
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

    % For documentation, see also:
    %
    % http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1Rotation.html#a600352007d9250f7f227f21db85611f2
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
            % Not a unique solution : roll − yaw = atan2(−R23,R22)
            rollPitchYaw(2) = pi/2;
            rollPitchYaw(3) =-atan2(-R(2,3),R(2,2));
            rollPitchYaw(1) = 0;
        end
    else
        % Not a unique solution : roll − yaw = atan2(−R23,R22)
        rollPitchYaw(2) = -pi/2;
        rollPitchYaw(3) = atan2(-R(2,3),R(2,2));
        rollPitchYaw(1) = 0;
    end
end
