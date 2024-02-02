function f_star = processOutputQP_twoFeet(analyticalSolution,primalSolution,QPStatus,Config)

    % PROCESSOUTPUTQP_TWOFEET evaluates the output of the WBToolbox QP block.
    %                         In case the QP block exited with an error, a
    %                        "default", user defined solution to the QP problem
    %                         is provided instead of the one coming from the QP block.
    %
    % FORMAT: f_star = processOutputQP_twoFeet(analyticalSolution,primalSolution,QPStatus,feetContactStatus,Config)
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
    QP_STATUS_THRESHOLD = 0.01;

    if Config.USE_QP_SOLVER && abs(QPStatus)< QP_STATUS_THRESHOLD

        f_star = primalSolution;
    else
        f_star = analyticalSolution;
    end
end
