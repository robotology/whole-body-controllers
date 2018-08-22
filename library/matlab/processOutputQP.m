function f0 = processOutputQP(analyticalSolution,primalSolution,QPStatus,Config)

tollQPStatus = 0.01;

if Config.USE_QP_SOLVER && abs(QPStatus)< tollQPStatus  
    
    f0 = primalSolution;
    
else    
    f0 = analyticalSolution;
end
end
    