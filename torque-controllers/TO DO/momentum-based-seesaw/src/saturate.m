function y = saturate(u, min, max)

assert(isequal(size(min), size(max)), 'Min and max must be same size')

if length(min) == 1
    y = u;
    y(y > max) = max;
    y(y < min) = min;
else 
    assert(length(min) == length(u), 'input and saturation must have same size');
 
    %i don't know how to do this one line..
    y = u;
    for i = 1:length(min)
        if y(i) > max(i)
            y(i) = max(i);
        elseif y(i) < min(i)
            y(i) = min(i);
        end
    end
   
end
