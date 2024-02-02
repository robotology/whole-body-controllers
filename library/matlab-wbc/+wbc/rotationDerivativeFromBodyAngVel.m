function RDot = rotationDerivativeFromBodyAngVel(omega,R)

    % ROTATIONDERIVATIVEFROMBODYANGVEL computes the derivative of a rotation matrix
    %                                  given the angular velocity in body coordinates.
    %                                  It makes use of the following convention: if
    %                                  the rotation of a body b is expressed in the
    %                                  world frame w (i.e., R = w_R_b), then this function
    %                                  is expecting the angular velocity of the body
    %                                  to be expressed in the body frame, i.e.
    %                                  omega = b_omega.
    %
    % USAGE: please note that this function has been designed for being inserted
    %        in a Simulink model.
    %
    % FORMAT: RDot = rotationDerivativeFromBodyAngVel(omega,R)
    %
    % INPUT:  - omega = [3 * 1] angular velocity
    %         - R     = [3 * 3] rotation matrix
    %
    % OUTPUT: - RDot  = [3 * 3] rotation matrix derivative
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    % all authors are with the Italian Istitute of Technology (IIT)
    % email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % Rotation matrix derivative (with correction to keep the integration
    % inside the space of rotation matrices). Omega is assumed to be w.r.t.
    % the body frame, i.e. b_omega.
    kCorr = 1;
    RDot  = R*wbc.skew(omega) +kCorr*(eye(3)-R*transpose(R))*R;
end
