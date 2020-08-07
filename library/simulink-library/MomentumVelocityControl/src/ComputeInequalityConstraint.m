function [C_in, b_in] = ComputeInequalityConstraint(C_frictionCones, b_frictionCones, NDOF)

    C_in = [zeros(size(C_frictionCones,1), 6+NDOF), C_frictionCones];
    b_in = b_frictionCones; 
end
