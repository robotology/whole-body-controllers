function [J_c, J_c_dot_nu] = ComputeContactJacobianWithFeetContactFCN(J_L, J_R, J_dot_nu_L, J_dot_nu_R, feet_contact_status)

NDOF = length(J_L(1,7:end));

J_c        = zeros(12,6+NDOF);
J_c_dot_nu = zeros(12,1);

% if left foot in contact
if(feet_contact_status(1))
    J_c (1:6,:)     = J_L;
    J_c_dot_nu(1:6) = J_dot_nu_L;
end

% if right foot in contact
if(feet_contact_status(2))
    J_c(7:end,:)      = J_R;
    J_c_dot_nu(7:end) = J_dot_nu_R;
end

end