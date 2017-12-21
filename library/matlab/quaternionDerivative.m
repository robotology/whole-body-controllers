% QUATERNIONDERIVATIVE computes the time derivative of a quaternion. 
%
% FORMAT: qtDot = quaternionDerivative(qt, omega, k)  
%
% INPUT:  - qt = [4 * 1] quaternion
%         - omega = [3 * 1] angular velocity
%         - k = gain for regularization terms
%
% OUTPUT: - qtDot = [4 * 1] quaternion derivative
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function qtDot = quaternionDerivative(qt, omega, k)

    qtDot = 0.5 *[0       -transpose(omega); ...
                  omega   -skew(omega)]*qt +k*(1 -transpose(qt)*qt)*qt;  
end
