function R = rotx(alpha)

    % ROTX computes a rotation along the x axis (rotation matrix).
    %
    % FORMAT: R = rotx(alpha)
    %
    % INPUT:  - alpha = angle in radians
    %
    % OUTPUT: - R = [3 * 3] rotation matrix along x axis
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---


    R =  [1, 0, 0;
        0, cos(alpha), -sin(alpha);
        0, sin(alpha), cos(alpha)];

end
