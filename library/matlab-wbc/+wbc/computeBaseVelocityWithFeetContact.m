function nu_b = computeBaseVelocityWithFeetContact(J_l_sole, J_r_sole, feetContactStatus, jointVel, pinvDampTolerance)

    % COMPUTEBASEVELOCITYWITHFEETCONTACT computes the floating base velocity assuming the
    %                                    left and/or right foot is in contact with the ground.
    %
    % FORMAT: nu_b = computeBaseVelocityWithFeetContact(J_l_sole, J_r_sole, feetContactStatus, jointVel, pinvDampTolerance)
    %
    % INPUT:  - J_l_sole          = [6 * ROBOT_DOF +6] left foot Jacobian
    %         - J_r_sole          = [6 * ROBOT_DOF +6] right foot Jacobian
    %         - feetContactStatus = [2 * 1] booleans describing the status of
    %                               the contacts (1 contact is active, 0 contact
    %                               is deactivated); [left; right]
    %         - jointVel          = [ROBOT_DOF * 1] joint velocity
    %         - pinvDampTolerance = tolerance for matrix pseudoinverse.
    %
    % OUTPUT: - nu_b              = [6 * 1] floating base velocity
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % Compute full contacts Jacobian
    Jc     = [feetContactStatus(1)*J_l_sole;
        feetContactStatus(2)*J_r_sole];

    % Compute multiplier of nu_b
    pinvJb =  wbc.pinvDamped(Jc(:,1:6), pinvDampTolerance);

    % Base velocity
    nu_b   = -pinvJb*Jc(:,7:end)*jointVel;
end
