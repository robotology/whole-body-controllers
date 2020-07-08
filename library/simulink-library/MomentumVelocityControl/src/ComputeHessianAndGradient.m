function [H_i, g_i] = ComputeHessianAndGradient(A_i,u_star,k_i)

H_i = k_i*(A_i')*A_i;
g_i = -2*k_i*(A_i')*u_star;

end