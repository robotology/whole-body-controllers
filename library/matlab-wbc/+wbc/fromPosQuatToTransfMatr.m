function H = fromPosQuatToTransfMatr(q)

    % FROMPOSQUATTOTRANSFMATRIX takes as input a pose (position + orientation)
    %                           with orientation expressed in quaternions, and
    %                           outputs the same pose in terms of transformation
    %                           matrix.
    %
    % FORMAT: H = fromPosQuatToTransfMatr(q)
    %
    % INPUT:   - q = [7 * 1] pose (position + quaternions)
    %
    % OUTPUT:  - H = [4 * 4] pose (transformation matrix)
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % extract the position
    x      = q(1:3);

    % compute the rotation matrix
    R      = wbc.rotationFromQuaternion(q(4:7));

    % compose the transformation matrix
    H      = [R,  x;
        [0 0 0 1]];
end
