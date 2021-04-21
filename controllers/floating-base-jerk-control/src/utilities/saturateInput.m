function y = saturateInput(u, umax, umin)

    % saturate the input values according to the specified limits
    y = zeros(length(u),1);
    
    for k = 1:length(u)
        
        if u(k) > umax(k)
            
            y(k) = umax(k);
            
        elseif u(k) < umin(k)
            
            y(k) = umin(k);
        else
            y(k) = u(k);
        end
    end
end