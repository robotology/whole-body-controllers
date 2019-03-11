function y = deadZone(u, thr)

    % DEADZONE implements a dead zone for the input u. If an element of u is 
    %          lower (absolute value) than a user-defined threshold,
    %          consider that element of u as zero. u can be either a vector
    %          or a scalar.
    %
    % FORMAT: y = deadZone(u, thr)
    %
    % INPUT:  - u   = [n * 1] input vector;
    %         - thr = user-defined threshold;
    %
    % OUTPUT: - y   = [n * 1] filtered output.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2018
    %

    %% --- Initialization ---
    
    % initialize output
    y = u;

    for i = 1:length(u)
    
        if abs(u(i)) < thr
        
            y(i) = 0;
        end
    end
end