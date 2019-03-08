function pose_bDot = basePoseDerivative(nu_b, pose_b)

    % BASEPOSEDERIVATIVE calculates the derivative of the floating base pose in
    %                    terms of position + quaternion derivative. The
    %                    resulting term is composed by the linear base 
    %                    velocities, and the derivative of the base 
    %                    orientation expressed in quaternions. 
    %
    % FORMAT: pose_bDot = basePoseDerivative(nu_b, pose_b)  
    %
    % INPUT:  - nu_b = [6 * 1] base velocity
    %         - pose_b = [7 * 1] base pose in quaternions
    %
    % OUTPUT: - pose_bDot = [7 * 1] base pose derivative expressed in terms
    %                        of linear velocities + quaternion derivative
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % base quaternion
    qt_b      = pose_b(4:end);
     
    % base rotation matrix
    w_R_b     = wbc.rotationFromQuaternion(qt_b);
     
    % express base velocity w.r.t. the base frame
    w_omega_b = nu_b(4:end);
    b_omega_b = transpose(w_R_b)*w_omega_b;
    
    % calculate the quaternion derivative
    k         = 1;
    qt_bDot   = wbc.quaternionDerivative(qt_b, b_omega_b, k);

    % compute the base pose derivative
    pose_bDot = [nu_b(1:3); qt_bDot];
end