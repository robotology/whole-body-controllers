% SATURATEINPUTDERIVATIVE Saturates the input u such that the absolute value
% of its numerical derivative uDelta = (uPrev-u)/Ts cannot be greater than
% a predefined value. uPrev = u at the previous integration step; Ts integration step.
%

%% --- Initialization ---
function uSat = saturateInputDerivative(u, u_0, Config)

persistent  uPrev;

% initialize the input value at the previous step
if isempty(uPrev)
    
    uPrev = u_0;
end

% max allowed input derivative
uDelta_maxAbs = Config.tauDot_maxAbs;

% evaluate the max and min allowed input
delta_u_max =  uDelta_maxAbs*Config.Ts; 
delta_u_min = -uDelta_maxAbs*Config.Ts;

delta_u_Sat = saturateInput(u-uPrev, delta_u_min, delta_u_max);

% update uPrev
uSat  = uPrev + delta_u_Sat;
uPrev = uSat;

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
