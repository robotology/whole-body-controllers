function [LDDot_des, LDot_des, L_des, intL_des] = computeMomentumReferences(pos_vel_acc_jerk_CoM_des, m)

    % get the CoM references for the jerk controller
    jerkCoM_des = pos_vel_acc_jerk_CoM_des(:,4);
    accCoM_des  = pos_vel_acc_jerk_CoM_des(:,3);
    velCoM_des  = pos_vel_acc_jerk_CoM_des(:,2);
    posCoM_des  = pos_vel_acc_jerk_CoM_des(:,1);
    
    % compute the linear momentum references. For the moment, angular
    % momentum references are set to zero by default
    LDDot_des   = [m*jerkCoM_des; zeros(3,1)];
    LDot_des    = [m*accCoM_des;  zeros(3,1)];
    L_des       = [m*velCoM_des;  zeros(3,1)];
    intL_des    = [m*posCoM_des;  zeros(3,1)];
end