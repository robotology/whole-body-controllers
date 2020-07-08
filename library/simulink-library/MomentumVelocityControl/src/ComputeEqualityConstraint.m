function [C_eq, b_eq] = ComputeEqualityConstraint(J_f, Jcmm, L_star)

C_eq = [J_f, zeros(12,12);...
        Jcmm, zeros(6,12)]; 
b_eq = [zeros(12,1); L_star]; 

end