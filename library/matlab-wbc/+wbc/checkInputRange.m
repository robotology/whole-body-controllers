function [inRange, res_check_range] = checkInputRange(umin, umax, u, tol)

    % CHECKINPUTRANGE checks if the current input value is inside the limits.
    %
    % FORMAT: [inRange, res_check_range] = checkInputRange(umin, umax, u, tol)
    %
    % INPUT:  - umin = [n * 1] min values;
    %         - umax = [n * 1] max values;
    %         - u    = [n * 1] current values;
    %         - tol  = tolerance;
    %
    % OUTPUT: - inRange = boolean for checking if u is inside the limits;
    %         - res_check_range = vector of booleans to check the single joints.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    res_check_range = u < umin + tol | u > umax - tol;
    res_tot         = sum(res_check_range);

    if res_tot == 0
        inRange = 1;
    else
        inRange = 0;
    end
end
