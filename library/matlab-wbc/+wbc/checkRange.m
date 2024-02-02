function inRange = checkRange(umin, umax, u, tol)

    % CHECKRANGE checks if the current input value is inside the limits.
    %
    % FORMAT: inRange = checkRange(umin, umax, u, tol)
    %
    % INPUT:  - umin = [n * 1] min values
    %         - umax = [n * 1] max values
    %         - u    = [n * 1] current values
    %         - tol  = tolerance
    %
    % OUTPUT: - inRange = boolean for checking if u is inside the limits
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % deprecation warning
    disp('WARNING: checkRange is deprecated and it will be removed in a future release. Use checkInputRange instead.')

    res  = u < umin + tol | u > umax - tol;
    res  = sum(res);

    if res == 0
        inRange = 1;
    else
        inRange = 0;
    end
end
