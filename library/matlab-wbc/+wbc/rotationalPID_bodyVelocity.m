function b_omega = rotationalPID_bodyVelocity(w_R_b, w_R_b_des, b_omega_des, Kp)

    % ROTATIONALPID_VELOCITY implements a trajectory tracking controller inside
    %                        the group SO(3). The angular velocity is assumed
    %                        to be a control input.
    %
    % FORMAT: b_omega = rotationalPID_velocity(w_R_b,w_R_b_des,b_omega_des,Kp)
    %
    % INPUT: - w_R_b       = [3 * 3] rotation matrix
    %        - w_R_b_des   = [3 * 3] desired rotation matrix
    %        - b_omega_des = [3 * 1] desired angular velocity (expressed in the body frame)
    %        - Kp          = [3 * 3] orientation gains
    %
    % OUTPUT: - b_omega    = [3 * 1] angular velocity in body coordinates
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    % all authors are with the Italian Istitute of Technology (IIT)
    % email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    skv     = wbc.skewVee(transpose(w_R_b_des)*w_R_b);
    b_omega = b_omega_des -Kp*skv;
end
