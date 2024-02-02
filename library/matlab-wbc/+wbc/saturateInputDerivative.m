function uSat = saturateInputDerivative(u, u_0, tStep, uDeltaMax)

    % SATURATEINPUTDERIVATIVE saturates the input u such that the absolute
    %                         value of its numerical derivative:
    %
    %                             uDelta = (uPrev-u)/tStep
    %
    %                         cannot be greater than a predefined value.
    %
    % FORMAT: uSat = saturateInputDerivative(u, u_0, Config, Sat)
    %
    % INPUTS: u = input signal;
    %         u_0 = input at t = 0;
    %         tStep = time step for finite difference formula;
    %         uDeltaMax = max input derivative (absolute value).
    %
    % OUTPUTS: uSat = saturated input signal.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---
    persistent  uPrev;

    % initialize the input value at the previous step
    if isempty(uPrev)

        uPrev   = u_0;
    end

    % evaluate the max and min allowed input
    delta_u_max =  uDeltaMax * tStep;
    delta_u_min = -uDeltaMax * tStep;
    delta_u_Sat = saturateInput(u-uPrev, delta_u_min, delta_u_max);

    % update u at previous time
    uSat        = uPrev + delta_u_Sat;
    uPrev       = uSat;
end

% utility function: saturates the input value
function y = saturateInput(u, min, max)

    assert(isequal(size(min), size(max)), 'Min and max must be same size')

    if length(min) == 1

        y          = u;
        y(y > max) = max;
        y(y < min) = min;
    else

        assert(length(min) == length(u), 'input and saturation must have same size');
        y = u;

        for i = 1:length(min)

            if y(i) > max(i)

                y(i) = max(i);

            elseif y(i) < min(i)

                y(i) = min(i);
            end
        end
    end
end
