% function [L_d_n, L_dot_d, int_L_ang, x_com_des, s_des_k, s_dot_des_k] = ComputeReferencesController(M,L,desired_pos_vel_COM, joint_pos, joint_pos_desired,joint_vel_desired, L_ang_des,L_d)

% persistent L_0; 
% 
% if(isempty(L_0))
%     L_0 = L; 
% end
% 
% persistent joint_pos_0; 
% 
% if(isempty(joint_pos_0))
%     joint_pos_0 = joint_pos; 
% end
% 
% L_d_n = zeros(6,1);
% L_dot_d   = zeros(6,1); 
% L_d_n(1:3)  = M(1,1)*desired_pos_vel_COM(:,2)';
% %L_d_n(4:6)  = L_d(4:6);
% int_L_ang    = zeros(3,1); 
% x_com_des    = desired_pos_vel_COM(:,1); 
% s_des_k      = joint_pos_desired; 
% s_dot_des_k  = joint_vel_desired; 
function [L_d_n, L_dot_d, int_L_ang, x_com_des, s_des_k, s_dot_des_k, vel_feet_correction] = ComputeReferencesController(M,L,desired_pos_vel_COM, joint_pos, joint_pos_desired,joint_vel_desired, L_ang_des,L_d, w_H_l, w_H_r,Cs, Gains)

persistent L_0; 

persistent w_H_l_0; 

persistent w_H_r_0; 

persistent z_0; 

persistent joint_pos_0; 

persistent Cs_old ;

if(isempty(L_0))
    L_0 = L; 
end

if(isempty(Cs_old))
    Cs_old = Cs; 
end

if(isempty(joint_pos_0))
    joint_pos_0 = joint_pos; 
end

%%TODO could also check on the contact status,to update it each time the
%%contact is restored. 


if (isempty(w_H_r_0))
    w_H_r_0 = w_H_r; 
end

if(isempty(w_H_l_0))
    w_H_l_0 = w_H_l; 
end

if(isempty(z_0))
    z_0 = w_H_l(3,4); 
end

% For now we are assuming that one foot is always in contact, we will
% discard this assumption for performing the jump 
% Updating only the position, we want to keep the original orientation 
if(~Cs_old(1) && Cs(1)) 
w_H_l_0(1,4) = w_H_l(1,4);
w_H_l_0(2,4) = w_H_l(2,4); 
w_H_l_0(3,4) = z_0; 
end

% Updating only the position, we want to keep the original orientation
if(~Cs_old(2) && Cs(2))
w_H_r_0(1,4) = w_H_r(1,4);
w_H_r_0(2,4) = w_H_r(2,4);
w_H_r_0(1:3,1:3)
w_H_r_0(3,4) = z_0; 
end

vel_feet_correction = zeros(12,1); 
if(Cs(1))
    vel_feet_correction(1:3) = diag(Gains.Kp_feet_correction)*(w_H_l(1:3,4)- w_H_l_0(1:3,4));
    vel_feet_correction(4:6) = Gains.Kp_feet_correction_angular* wbc.skewVee(w_H_l(1:3,1:3)*transpose(w_H_l_0(1:3,1:3)));
end

if(Cs(2))
    vel_feet_correction(7:9) = diag(Gains.Kp_feet_correction)*(w_H_r(1:3,4)- w_H_r_0(1:3,4));
    vel_feet_correction(10:12) = Gains.Kp_feet_correction_angular* wbc.skewVee(w_H_r(1:3,1:3)*transpose(w_H_r_0(1:3,1:3)));
end
L_d_n = zeros(6,1);
L_dot_d   = zeros(6,1); 
L_d_n(1:3)  = M(1,1)*desired_pos_vel_COM(:,2)';
%L_d_n(4:6)  = L_d(4:6);
int_L_ang    = zeros(3,1); 
x_com_des    = desired_pos_vel_COM(:,1); 
s_des_k      = joint_pos_desired; 
s_dot_des_k  = joint_vel_desired; 

Cs_old = Cs; 

end

