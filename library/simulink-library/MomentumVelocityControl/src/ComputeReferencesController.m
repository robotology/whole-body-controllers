function [L_d, L_dot_d, int_L_ang, x_com_des, s_des_k, s_dot_des_k] = ComputeReferencesController(M,L,desired_pos_vel_COM, joint_pos, joint_pos_desired, L_ang_des,L_d)

persistent L_0; 

if(isempty(L_0))
    L_0 = L; 
end

persistent joint_pos_0; 

if(isempty(joint_pos_0))
    joint_pos_0 = joint_pos; 
end


%% for balancing 
%L_d          = L_0;

% L_d = zeros(6,1); 
%L_d(4:6)= L_ang_des;
L_dot_d   = zeros(6,1); 
L_d(1:3)  = M(1,1)*desired_pos_vel_COM(:,2)';
int_L_ang    = zeros(3,1); 
x_com_des    = desired_pos_vel_COM(:,1); 
s_des_k      = joint_pos_desired; 
s_dot_des_k  = zeros(size(s_des_k)); 


end
