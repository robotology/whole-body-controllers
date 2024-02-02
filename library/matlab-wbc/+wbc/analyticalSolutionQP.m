function analyticalSolution = analyticalSolutionQP(HessianMatrixQP,gradientQP)

    % ANALYTICALSOLUTIONQP provides the unconstrained solution of a QP
    %                      problem. To be used as possible alternative when
    %                      the WBToolbox "QP block" fails to find a solution.
    %
    % FORMAT: analyticalSolution = analyticalSolutionQP(HessianMatrixQP,gradientQP)
    %
    % INPUT:   - HessianMatrixQP = hessian matrix of the QP problem;
    %          - gradientQP      = gradient of the QP problem.
    %
    % OUTPUT:  - analyticalSolution = the analytical solution of the QP problem.
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
