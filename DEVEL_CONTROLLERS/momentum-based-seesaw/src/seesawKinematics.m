function [J_s,J_sDot_nu_s,J_f,J_fDot_nu_s,s_p_CoM_s] = seesawKinematics(seesaw,s_omega,w_R_s,w_H_lSole)

% Kinematics of the seesaw board, projected in a frame attached to the seesaw

% Parameters from seesaw 
delta = seesaw.delta;
rho = seesaw.rho;
s_sr = seesaw.s_sr;
s_sl = seesaw.s_sl;

% Constants
e3 = [0;0;1];
e2 = [0;1;0];

% Common variables in world frame
w_r = (delta * w_R_s * e3) -rho * e3;
w_sl = w_R_s * s_sl;
w_sr = w_R_s * s_sr;
w_p_lSole = w_H_lSole(1:3,4);
w_R_s_bar = blkdiag(w_R_s, w_R_s);
w_RDot_s_bar = w_R_s_bar * blkdiag(skew(s_omega),skew(s_omega));

% Common variables in seesaw frame
s_R_w = transpose(w_R_s);
s_r = s_R_w * w_r;
s_v_s = skew(s_r) * s_omega;
s_nu_s = [s_v_s; s_omega];
s_rDot = rho * skew(s_omega) * s_R_w * e3;

% Jacobian matrix for seesaw constraints (rolling + no rotation along y-z axis)
J_s = [eye(3)     -skew(s_r);
       zeros(1,3)  transpose(e2)*w_R_s;
       zeros(1,3)  transpose(e3)*w_R_s;];

% Derivative of J_s times nu_s
J_sDot_nu_s = [zeros(3)   -skew(s_rDot);
               zeros(1,3)  transpose(e2)*w_R_s*skew(s_omega);
               zeros(1,3)  transpose(e3)*w_R_s*skew(s_omega);] * s_nu_s;

% Jacobian matrix that maps seesaw velocity in seesaw frame to feet velocity
J_f = [eye(3)    -skew(w_sl);
       zeros(3)   eye(3);
       eye(3)    -skew(w_sr);
       zeros(3)   eye(3)] * w_R_s_bar;

% Time derivative of J_f times nu_s
J_fDot_1 = [zeros(3)  -skew(w_R_s * skew(s_omega) * s_sl);
            zeros(3)   zeros(3);
            zeros(3)  -skew(w_R_s * skew(s_omega) * s_sr);
            zeros(3)   zeros(3)] * w_R_s_bar;
           
J_fDot_2 = J_f * w_RDot_s_bar;

J_fDot_nu_s = (J_fDot_1 + J_fDot_2) * s_nu_s; 

% Seesaw CoM in seesaw frame
s_p_CoM_s = s_R_w * (w_p_lSole - w_sl);
 
end
