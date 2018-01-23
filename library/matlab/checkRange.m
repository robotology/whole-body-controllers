% CHECKRANGE checks if the current joint position is inside the limits.
%

function inRange = checkRange(umin, umax, u, tol)

    res = u < umin + tol | u > umax - tol;
    res = sum(res);
    
    if res==0
        inRange = 1;
    else
        inRange = 0;
    end
end