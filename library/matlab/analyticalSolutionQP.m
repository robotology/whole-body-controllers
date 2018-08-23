function analyticalSolution = analyticalSolutionQP(HessianMatrixQP,gradientQP)

    analyticalSolution = -inv(HessianMatrixQP)*gradientQP;
end