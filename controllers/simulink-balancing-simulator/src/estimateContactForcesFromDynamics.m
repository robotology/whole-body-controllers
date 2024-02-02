function f_dynamics = estimateContactForcesFromDynamics(tau, JL, JR, JDotL_nu, JDotR_nu, M, h, LEFT_RIGHT_FOOT_IN_CONTACT)

    % try to estimate the contact forces using the joint torques, the
    % dynamics model, the robot state and the contact constraints.
    ndof       = size(M,1)-6;
    B          = [zeros(6,ndof); eye(ndof)];
    f_dynamics = zeros(12,1);

    if sum(LEFT_RIGHT_FOOT_IN_CONTACT) > 1.5

        % two feet balancing
        Jc          = [JL; JR];
        JcDot_nu    = [JDotL_nu; JDotR_nu];

        pinvDampJMJ = eye(12)/(Jc/M*Jc');
        f_dynamics  = pinvDampJMJ*(Jc/M*(h-B*tau)-JcDot_nu);

    elseif LEFT_RIGHT_FOOT_IN_CONTACT(1) > 0.5 && LEFT_RIGHT_FOOT_IN_CONTACT(2) < 0.5

        % left foot balancing
        Jc_left          = JL;
        Jc_leftDot_nu    = JDotL_nu;

        pinvDampJMJ_left = eye(6)/(Jc_left/M*Jc_left');
        f_dynamics       = [pinvDampJMJ_left*(Jc_left/M*(h-B*tau)-Jc_leftDot_nu); zeros(6,1)];

    elseif LEFT_RIGHT_FOOT_IN_CONTACT(2) > 0.5 && LEFT_RIGHT_FOOT_IN_CONTACT(1) < 0.5

        % right foot balancing
        Jc_right          = JR;
        Jc_rightDot_nu    = JDotR_nu;

        pinvDampJMJ_right = eye(6)/(Jc_right/M*Jc_right');
        f_dynamics        = [zeros(6,1); pinvDampJMJ_right*(Jc_right/M*(h-B*tau)-Jc_rightDot_nu)];
    end
end
