function R = rotz(alpha)

    % ROTZ computes a rotation along the z axis (rotation matrix).
    %
    % FORMAT: R = rotz(alpha)     
    %
    % INPUT:  - alpha = angle in radians
    %
    % OUTPUT: - R = [3 * 3] rotation matrix along z axis
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

   R = [cos(alpha), -sin(alpha), 0;
        sin(alpha),  cos(alpha), 0;
        0,           0,          1];

end
