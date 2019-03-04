function M_with_inertia = addMotorsReflectedInertia(M,Gamma,T,I_m)

    % ADDMOTORSREFLECTEDINERTIA adds the motors reflected inertias to the
    %                           joint space mass matrix.
    %
    % PROCEDURE: the assume the floating base + joint + motors dynamics can be
    % described by the following system of equations:
    %
    %     M_b*nuDot_b + M_bs*sDDot + h_b = J_b^T*f (1)
    %
    %     M_s*sDDot + M_sb*nuDot_b + h_s = J_s^T*f + tau_s (2)
    %
    %     I_m*thetaDDot = tau_m - transpose(T*Gamma)*tau_s (3)
    %
    % Assuming that the joints are rigidly attached to the motors, one has that:
    %
    %     thetaDDot = (T*Gamma)^-1*sDDot (4)
    %
    % where Gamma is a diagonal matrix containing the transmission ratio for
    % all joints, while T is a block diagonal matrix taking into account the 
    % fact that some joint movements are obtained by combining the effect of 
    % different motors (see also http://wiki.icub.org/wiki/ICub_coupled_joints).
    % By substituting (4) into (3), multiplying (3) by transpose(T*Gamma)^-1 and
    % finally summing up (3) and (2), one has:
    %
    %     M_b*nuDot_b + M_bs*sDDot + h_b = J_b^T*f (1)
    %
    %    (M_s+transpose(T*Gamma)^-1*I_m*(T*Gamma)^-1)*sDDot + M_sb*nuDot_b + h_s = J_s^T*f + u (2b)
    %
    % where u = transpose(T*Gamma)^-1*tau_m. The input joint torques can be 
    % calculated from (3) as follows:
    %
    %    tau_s = u - K_ff*transpose(T*Gamma)^-1*I_m*(T*Gamma)^-1*sDDot 
    %
    % with K_ff belonging to [0,1].
    %
    % FORMAT: M_with_inertia = addMotorsReflectedInertia(M,Gamma,T,I_m)
    %
    % INPUT:  - M     = [6+n x 6+n] mass matrix;
    %         - Gamma = [n x n] diagonal matrix that accounts for the transmission 
    %                           ratio of the joints in the mechanism;
    %         - T     = [n x n] matrix that accounts for the coupling between
    %                           different joints;
    %         - I_m   = [n x n] diagonal matrix that contains the motors
    %                           inertia (not reflected).
    %
    % OUTPUT: - M_with_inertia = [6+n x 6+n] mass matrix with motor reflected
    %                             inertia.
    %
    % Authors: Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---
    
    % parameters
    NDOF = size(M,1)-6;
      
    reflectedInertia    = wbc.computeMotorsReflectedInertia(Gamma,T,I_m);
        
    M_reflected_inertia = [zeros(6,6+NDOF);
                           zeros(NDOF,6) reflectedInertia];

    M_with_inertia      = M + M_reflected_inertia;
end