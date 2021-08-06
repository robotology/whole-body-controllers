function [noSpikes, res_check_spikes] = checkInputSpikes(u, delta_u_max)

    % CHECKINPUTSPIKES checks the (unsigned) difference between two consecutive 
    %                  measurements of the input u, i.e. delta = abs(u(k)-u(k-1))
    %                  returns FALSE if delta is bigger than a user-defined threshold.
    %
    % FORMAT: [noSpikes, res_check_spikes] = checkInputSpikes(u, delta_u_max)
    %
    % INPUT:  - u = [n * 1] input values;
    %         - delta_u_max = user-defined threshold;
    %
    % OUTPUT: - noSpikes = FALSE if delta is greater than delta_u_max;
    %         - res_check_spikes = vector of booleans to check the single joints.
    %
    % Authors: Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2018
    %

    %% --- Initialization ---

    persistent u_previous
    
    if isempty(u_previous)
        
        u_previous = u;
    end
      
    delta = abs(u - u_previous);
    
    % u may be a vector
    res_check_spikes = delta >= delta_u_max;
    res_tot          = sum(res_check_spikes);
    
    if abs(res_tot) < 0.1
        
        noSpikes = 1;
    else
        noSpikes = 0;
    end
    
    % now, set u_previous to be u
    u_previous = u;
end