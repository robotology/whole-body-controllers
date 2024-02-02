function nuDot = forwardDynamics(tau, JL, JR, M, h, f, LEFT_RIGHT_FOOT_IN_CONTACT)

    % get the robot accelerations using the joint torques, the
    % dynamics model, the robot state and the contact constraints.
    ndof  = size(M,1)-6;
    B     = [zeros(6,ndof); eye(ndof)];

    nuDot = M\(B*tau+LEFT_RIGHT_FOOT_IN_CONTACT(1)*JL'*f(1:6)+LEFT_RIGHT_FOOT_IN_CONTACT(2)*JR'*f(7:end)-h);

end
