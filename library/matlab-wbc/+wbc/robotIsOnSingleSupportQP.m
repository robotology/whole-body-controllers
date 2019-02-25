function onOneFoot = robotIsOnSingleSupportQP(feetContactStatus)

    % ROBOTISONSINGLESUPPORTQP detects if the robot is balancing on single
    %                          or double support.
    %                                        
    % FORMAT: onOneFoot = robotIsOnSingleSupportQP(feetContactStatus)
    %
    % INPUT:   - feetContactStatus = [2 x 1] a vector describing the feet contact 
    %                                status. Format: [leftFoot; rightFoot];
    %
    % OUTPUT:  - onOneFoot = true if the robot is balancing on one foot, false
    %                        otherwise.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    CONTACT_THRESHOLD = 0.1;

    if sum(feetContactStatus) > (2 - CONTACT_THRESHOLD)
        
        % two feet balancing
        onOneFoot = false;
        return;
        
    elseif sum(feetContactStatus) > (1 - CONTACT_THRESHOLD)
        
        % one foot balancing
        onOneFoot = true;
        return;
    else
        onOneFoot = false;
        return;
    end
end