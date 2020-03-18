function noSpikes = checkSpikes(u, delta_u_max)

    % CHECKSPIKES checks the (unsigned) difference between two consecutive 
    %             measurements of the input u, i.e. delta = abs(u(k)-u(k-1))
    %             returns FALSE if delta is bigger than a user-defined threshold.
    %
    % FORMAT: inRange = checkSpikes(u, delta_u_max)  
    %
    % INPUT:  - u = [n * 1] input values;
    %         - delta_u_max = user-defined threshold;
    %
    % OUTPUT: - inRange = FALSE if delta is greater than delta_u_max.
    %
    % Authors: Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2018
    %

    %% --- Initialization ---

    % deprecation warning
    disp('WARNING: checkSpikes is deprecated and it will be removed in a future release. Use checkInputSpikes instead.')
    
    persistent u_previous
    
    if isempty(u_previous)
        
        u_previous = u;
    end
      
    delta = abs(u - u_previous);
    
    % u may be a vector
    res  = delta >= delta_u_max;
    res  = sum(res);
    
    if abs(res) < 0.1
        
        noSpikes = 1;
    else
        noSpikes = 0;
    end
    
    % now, set u_previous to be u
    u_previous = u;
end