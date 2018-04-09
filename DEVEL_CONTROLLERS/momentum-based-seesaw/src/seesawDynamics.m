function [M_s,h_s,H_s] = seesawDynamics(seesaw,s_omega,w_R_s)

% Dynamics of the seesaw board, projected in a frame attached to the seesaw

% Parameters from seesaw 
m_s = seesaw.mass;
I_s = seesaw.inertia;
delta = seesaw.delta;
rho = seesaw.rho;

% Common variables and parameters
s_R_w  = transpose(w_R_s);
e3 = [0;0;1];
g = -9.81;
s_g = s_R_w * g * e3;
s_r = (delta * e3) -(s_R_w * rho * e3);
s_v_s = skew(s_r) * s_omega;
s_nu_s = [s_v_s; s_omega];

% Seesaw mass matrix
M_s = [m_s*eye(3) zeros(3);
         zeros(3)  I_s];   
     
% Seesaw Coriolis and gravity terms
h_s = [(-m_s*s_g + skew(s_omega)*m_s*s_v_s);
        skew(s_omega)*I_s*s_omega];
 
% Seesaw linear and angular momentum in seesaw frame
H_s = M_s * s_nu_s;

end