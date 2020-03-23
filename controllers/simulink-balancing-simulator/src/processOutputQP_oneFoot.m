function f_star = processOutputQP_oneFoot(analyticalSolution,primalSolution,QPStatus,feetContactStatus,Config)

    % PROCESSOUTPUTQP_ONEFOOT evaluates the output of the WBToolbox QP block.
    %                         In case the QP block exited with an error, a 
    %                        "default", user defined solution to the QP problem 
    %                         is provided instead of the one coming from the QP block. 
    %
    % FORMAT: f_star = processOutputQP_oneFoot(analyticalSolution,primalSolution,QPStatus,feetContactStatus,Config)  
    %
    % INPUT:  - analyticalSolution = the alternative user defined solution to the QP 
    %                                problem to be used when the QP block fails;
    %         - primalSolution = the solution to the QP problem provided by the 
    %                            WBToolbox QP block;
    %         - QPStatus = the status check returned by the QP block.
    %
    %         - feetContactStatus = [2 x 1] a vector describing the feet contact 
    %                               status. Format: [leftFoot; rightFoot];
    %         - Config = a structure containing the robot-related configuration
    %                    parameters;
    %
    % OUTPUT: - f_star = the final solution to the QP problem that will be used in
    %                    the controller.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    CONTACT_THRESHOLD   = 0.1;
    QP_STATUS_THRESHOLD = 0.01;

    % if the robot is balancing on one foot, extend the QP solution to
    % have the correct dimension of f_star for matrix multiplication  
    if feetContactStatus(1) > (1 - CONTACT_THRESHOLD)
        
        % left foot balancing
        updated_primalSolution     = [primalSolution; zeros(6,1)];
        updated_analyticalSolution = [analyticalSolution; zeros(6,1)];
  
    else       
        % right foot balancing
        updated_primalSolution     = [zeros(6,1); primalSolution];
        updated_analyticalSolution = [zeros(6,1); analyticalSolution]; 
    end
    
    if Config.USE_QP_SOLVER && abs(QPStatus)< QP_STATUS_THRESHOLD  
    
        f_star = updated_primalSolution;  
    else    
        f_star = updated_analyticalSolution;
    end
end