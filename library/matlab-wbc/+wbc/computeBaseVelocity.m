function nu_b = computeBaseVelocity(J_l_sole, J_r_sole, feetContactStatus, jointVel, Reg)

    % COMPUTEBASEVELOCITY computes the floating base velocity assuming the
    %                     left/right foot is in contact with the ground.
    %
    % FORMAT: nu_b = computeBaseVelocity(J_l_sole, J_r_sole, feetContactStatus, jointVel, Reg)
    %
    % INPUT:  - J_l_sole = [6 * ROBOT_DOF +6] left foot Jacobian
    %         - J_r_sole = [6 * ROBOT_DOF +6] right foot Jacobian
    %         - feetContactStatus = [2 * 1] feet in contact
    %         - jointVel = [ROBOT_DOF * 1] joint velocity
    %         - Reg = regularization parameters
    %
    % OUTPUT: - nu_b = [6 * 1] floating base velocity
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %
 
    %% --- Initialization ---

    % Compute Jacobian
    Jc     = [feetContactStatus(1)*J_l_sole;
              feetContactStatus(2)*J_r_sole];

    % Compute multiplier of nu_b  
    pinvJb =  wbc.pinvDamped(Jc(:,1:6), Reg.pinvDamp_nu_b);  
  
    % Base velocity
    nu_b   = -pinvJb*Jc(:,7:end)*jointVel;
end