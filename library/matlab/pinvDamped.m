% PINVDAMPED computes the damped pseudoinverse of matrix A
%
function pinvDampA = pinvDamped(A,regDamp)

    pinvDampA = A'/(A*A' + regDamp*eye(size(A,1)));
    
end