function analyticalSolution = analyticalSolutionQP(HessianMatrixQP,gradientQP)

    % ANALYTICALSOLUTION resolves the unconstrained solution to a QP problem
    %                    (to be used as possible alternative when the WBToolbox
    %                    QP block fails to find a solution)
    %                                        
    % FORMAT: analyticalSolution = analyticalSolutionQP(HessianMatrixQP,gradientQP)
    %
    % INPUT:   - HessianMatrixQP = hessian matrix of the QP problem;
    %          - gradientQP = gradient of the QP problem.
    %
    % OUTPUT:  - analyticalSolution = the analytical solution to the QP problem.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    analyticalSolution = -inv(HessianMatrixQP)*gradientQP;
end
