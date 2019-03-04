function [pos_l_sole_error, rot_l_sole_error, pos_r_sole_error, rot_r_sole_error, posCoM] = ...
             computePositionAndOrientationErrors(w_H_l_sole, w_H_r_sole, w_H_CoM, w_H_r_sole_0, w_H_l_sole_0)

    % COMPUTEPOSITIONANDORIENTATIONERRORS retrieves the feet and CoM
    %                                     position and orientation errors.
    
    %% --- Initialization ---
    
    % position errors
    posCoM           = w_H_CoM(1:3,4);
    pos_l_sole       = w_H_l_sole(1:3,4);
    pos_r_sole       = w_H_r_sole(1:3,4);
    pos_l_sole_0     = w_H_l_sole_0(1:3,4);
    pos_r_sole_0     = w_H_r_sole_0(1:3,4);
    pos_l_sole_error = pos_l_sole - pos_l_sole_0;
    pos_r_sole_error = pos_r_sole - pos_r_sole_0;

    % rotation matrices at feet
    w_R_l_sole       = w_H_l_sole(1:3,1:3);
    w_R_r_sole       = w_H_r_sole(1:3,1:3);
    w_R_l_sole_0     = w_H_l_sole_0(1:3,1:3);
    w_R_r_sole_0     = w_H_r_sole_0(1:3,1:3);

    % feet orientation is parametrized with Euler angles
    rot_l_sole       = wbc.rollPitchYawFromRotation(w_R_l_sole);
    rot_r_sole       = wbc.rollPitchYawFromRotation(w_R_r_sole);
    rot_l_sole_0     = wbc.rollPitchYawFromRotation(w_R_l_sole_0);
    rot_r_sole_0     = wbc.rollPitchYawFromRotation(w_R_r_sole_0);
    
    % feet orientation errors
    rot_l_sole_error = rot_l_sole - rot_l_sole_0;
    rot_r_sole_error = rot_r_sole - rot_r_sole_0;
end