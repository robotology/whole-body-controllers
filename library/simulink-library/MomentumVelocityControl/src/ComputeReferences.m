function [L_star, s_dot_star_k_1] = ComputeReferences(x_com, x_d_com, Ld, int_L_angular, s_dot_des_k_1, s_des_k, s_k, Gains)

L_int_lin = Gains.KP_CoM* (x_com- x_d_com); 
L_int_ang = Gains.KI_ang* (int_L_angular); 
L_int = [L_int_lin; L_int_ang]; 

L_star = Ld - L_int; 

s_dot_star_k_1 = s_dot_des_k_1 - Gains.KP_Postural*(s_k- s_des_k); 

end