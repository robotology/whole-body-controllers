function H = fromPosRpyToTransfMatrix(pos_rpy)

    % FROMPOSRPYTOTRANSFMATRIX takes as input a pose (position + orientation)
    %                           with orientation expressed in roll-pitch-yaw, and
    %                           outputs the same pose in terms of transformation
    %                           matrix.
    %
    % FORMAT: H = fromPosRpyToTransfMatrix(pos_rpy)
    %
    % INPUT:  - pos_rpy = [6 * 1] pose (position + roll-pitch-yaw)
    %
    % OUTPUT: - H = [4 * 4] pose (transformation matrix)
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % following the compostion rule for rotation matrix as explained here:
    % http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1Rotation.html#a600352007d9250f7f227f21db85611f2
    rpy = pos_rpy(4:6);
    R   = wbc.rotz(rpy(3))*wbc.roty(rpy(2))*wbc.rotx(rpy(1));

    pos = pos_rpy(1:3);

    % compose the transformation matrix
    H   = [R,  pos;
        [0 0 0 1]];
end
