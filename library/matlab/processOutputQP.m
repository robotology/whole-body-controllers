function f0 = processOutputQP(analyticalSolution,primalSolution,QPStatus,Config)

    % PROCESSOUTPUTQP evaluates the output of the WBToolbox QP block. In case
    % the QP block exited with an error, a "default", user defined solution to 
    % the QP problem is provided instead of the one coming from the QP block. 
    %
    % FORMAT: f0 = processOutputQP(analyticalSolution,primalSolution,QPStatus,Config)   
    %
    % INPUT:  - analyticalSolution = the alternative user defined solution to the QP 
    %                                problem to be used when the QP block fails;
    %         - primalSolution = the solution to the QP problem provided by the 
    %                            WBToolbox QP block;
    %         - QPStatus = the status check returned by the QP block.
    %         - Config = a structure containing the robot-related configuration
    %                    parameters;
    %
    % OUTPUT: - f0 = the final solution to the QP problem that will be used in
    %                the controller.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    tollQPStatus = 0.01;

    if Config.USE_QP_SOLVER && abs(QPStatus)< tollQPStatus  
    
        f0 = primalSolution;  
    else    
        f0 = analyticalSolution;
    end
end
    
