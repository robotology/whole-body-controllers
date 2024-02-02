function R = rotationFromRollPitchYaw(rpy)

    % ROTATIONFROMROLLPITCHYAW converts Euler angles (roll-pitch-yaw convention)
    %                          into a rotation matrix.
    %
    % FORMAT:  R = rotationFromRollPitchYaw(rpy)
    %
    % INPUTS:  - rpy = [3 * 1] roll-pitch-yaw vector;
    %
    % OUTPUTS: - R = [3 * 3] rotation matrix.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Dec 2018

    %% ------------Initialization----------------

    % composition rule for rotation matrices explained here:
    %
    % http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1Rotation.html#a600352007d9250f7f227f21db85611f2
    %
    R   = wbc.rotz(rpy(3))*wbc.roty(rpy(2))*wbc.rotx(rpy(1));
end
