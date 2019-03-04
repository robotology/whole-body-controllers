function nuDot_ikin = computeStateAccelerations(pos_l_sole_error, pos_r_sole_error, rot_r_sole_error, rot_l_sole_error, ...
                                                posCoM, jointPos, desired_pos_vel_acc_CoM, desired_pos_vel_acc_joints, vel_l_sole, vel_r_sole, ...
                                                velCoM, jointVel, J_l_sole, J_r_sole, J_CoM, JDot_l_sole_nu, JDot_r_sole_nu, JDot_CoM_nu, feetcontactStatus, Config)
                                            
    % COMPUTESTATEACCELERATIONS computes the floating base reference state
    %                           accelerations by using a three task stack
    %                           of task inverse kinematics approach.
     
    %% --- Initialization ---
    
    % internal parameters
    pinv_tol      = Config.Reg.pinv_tol;  
    ndof          = size(J_CoM(:,7:end),2);

    % feet accelerations
    accFeet_star  = [(-Config.Gain.KP_feet * pos_l_sole_error(1:3) - Config.Gain.KD_feet * vel_l_sole(1:3));
                     (-Config.Gain.KP_feet * rot_l_sole_error(1:3) - Config.Gain.KD_feet * vel_l_sole(4:6));
                     (-Config.Gain.KP_feet * pos_r_sole_error(1:3) - Config.Gain.KD_feet * vel_r_sole(1:3));
                     (-Config.Gain.KP_feet * rot_r_sole_error(1:3) - Config.Gain.KD_feet * vel_r_sole(4:6))];
               
    % CoM accelerations                                       
    accCoM_star   = desired_pos_vel_acc_CoM(:,3) -Config.Gain.KP_CoM*(posCoM - desired_pos_vel_acc_CoM(:,1)) -Config.Gain.KD_CoM * (velCoM-desired_pos_vel_acc_CoM(:,2));

    % Joints accelerations
    accJoint_star = desired_pos_vel_acc_joints(:,3) -Config.Gain.KP_joints*(jointPos - desired_pos_vel_acc_joints(:,1)) -Config.Gain.KD_joints*(jointVel - desired_pos_vel_acc_joints(:,2));

    % Update Jacobians
    J_feet        = [feetcontactStatus(1)*J_l_sole ; feetcontactStatus(2)*J_r_sole];
    J_CoM         = J_CoM(1:3,:);
    J_postural    = [zeros(ndof,6), eye(ndof)]; 
    JDot_feet_nu  = [feetcontactStatus(1)*JDot_l_sole_nu; feetcontactStatus(2)*JDot_r_sole_nu];
    JDot_CoM_nu   = JDot_CoM_nu(1:3);

    % Null space projectors for primary and secondary task
    Null_feet     = eye(6+ndof) -pinv(J_feet, pinv_tol)*J_feet;
    Null_CoM      = eye(6+ndof) -pinv(J_CoM*Null_feet, pinv_tol)*J_CoM*Null_feet;

    %% Stack of tasks inverse kinematics. 

    % Primary task: respect the constraints at feet
    nuDot_feet    = pinv(J_feet, pinv_tol)*(accFeet_star -JDot_feet_nu);

    % Secondary task: achieve a desired CoM dynamics
    nuDot_CoM     = pinv(J_CoM*Null_feet, pinv_tol)*(accCoM_star -JDot_CoM_nu -J_CoM*nuDot_feet);

    % Third task: achieve a desired posture
    nuDot_joint   = pinv(J_postural*Null_feet*Null_CoM, pinv_tol)*(accJoint_star -J_postural*nuDot_feet -J_postural*Null_feet*nuDot_CoM);

    % Floating base and joints accelerations
    nuDot_ikin    = nuDot_feet + Null_feet*(nuDot_CoM + Null_CoM*nuDot_joint);
end