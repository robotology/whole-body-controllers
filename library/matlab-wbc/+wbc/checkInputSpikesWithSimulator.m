function [noSpikes, res_check_spikes] = checkInputSpikesWithSimulator(u_prev, u, delta_u_max)

    % CHECKINPUTSPIKESWITHSIMULATOR checks the (unsigned) difference between two consecutive 
    %                               measurements of the input u, i.e. delta = abs(u(k)-u(k-1))
    %                               returns FALSE if delta is bigger than a user-defined threshold.
    %
    % FORMAT:  [noSpikes, res_check_spikes] = checkInputSpikesWithSimulator(u_prev, u, delta_u_max)
    %
    % INPUT:  - u = [n x 1] input values;
    %         - u_prev = delayed [n x 1] input values;
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
    
    delta = abs(u - u_prev);
    
    % u may be a vector
    res_check_spikes = (delta >= delta_u_max);
    noSpikes         = double(~any(res_check_spikes));
end
