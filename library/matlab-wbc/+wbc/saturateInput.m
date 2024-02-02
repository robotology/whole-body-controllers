function y = saturateInput(u, umin, umax)

    % SATURATEINPUT saturates the input value.
    %
    % FORMAT: y = saturateInput(u, umin, umax)
    %
    % INPUT:  - umin = [n * 1] min values;
    %         - umax = [n * 1] max values;
    %         - u    = [n * 1] current values;
    %
    % OUTPUT: - y = [n * 1] saturated values;
    %
    % Author(s): Gabriele Nava
    %
    %            all authors are with the Italian Istitute of Technology (IIT)
    %            email: name.surname@iit.it
    %
    % Genoa, Jul 2020
    %

    %% --- Initialization ---
    assert(isequal(size(umin), size(umax)), 'umin and umax must be same size')

    if length(umin) == 1

        y           = u;
        y(y > umax) = umax;
        y(y < umin) = umin;

    else
        assert(length(umin) == length(u), 'input and saturation must have same size');
        y = u;

        for i = 1:length(umin)

            if y(i) > umax(i)

                y(i) = umax(i);

            elseif y(i) < umin(i)

                y(i) = umin(i);
            end
        end
    end
end
