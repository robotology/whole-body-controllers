% SKEWVEE generates a [3 * 1] vector from a [3 * 3] matrix. In particular,
%         this function does two operations: first, it creates the skew 
%         symmetric matrix of the form (X - X')/2, and then retrives from 
%         this skew symmetric matrix the [3 * 1] vector composing it.
%
% FORMAT: x = skewVee(X)  
%
% INPUT:  - X = [3 * 3] matrix
%
% OUTPUT: - x = [3 * 1] vector 
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function x = skewVee(X)

    % skew symmetric part of X
    X_skew = 0.5*(X -transpose(X)); 
    
    % vector composing the matrix
    x = [-X_skew(2,3)
          X_skew(1,3)
         -X_skew(1,2)];
end

