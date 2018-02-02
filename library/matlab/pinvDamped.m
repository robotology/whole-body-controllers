% PINVDAMPED computes the damped pseudoinverse of matrix A
%
% FORMAT: pinvDampA = pinvDamped(A,regDamp)   
%
% INPUT:  - A = [n * m] rotation matrix
%         - regDamp = regularization parameter
%
% OUTPUT: - pinvDampA = [m * n] matrix pseudoinverse of A
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function pinvDampA = pinvDamped(A,regDamp)

    pinvDampA = transpose(A)/(A*transpose(A) + regDamp*eye(size(A,1)));
    
end