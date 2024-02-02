function R = roty(alpha)

    % ROTY computes a rotation along the y axis (rotation matrix).
    %
    % FORMAT: R = roty(alpha)
    %
    % INPUT:  - alpha = angle in radians
    %
    % OUTPUT: - R = [3 * 3] rotation matrix along y axis
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    R = [cos(alpha), 0, sin(alpha);
        0, 1, 0;
        -sin(alpha), 0, cos(alpha)];

end
