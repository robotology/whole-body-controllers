function [A_p, a_p] = ComputePosturalTask(s_star_k_1, NDOF)

    A_p = [zeros(NDOF,6), eye(NDOF,NDOF), zeros(NDOF,12)]; 
    a_p = s_star_k_1; 
end
