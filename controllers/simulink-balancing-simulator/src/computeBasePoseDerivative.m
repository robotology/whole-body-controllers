function pose_base_dot = computeBasePoseDerivative(nu_base_ikin, pose_base)

    % COMPUTEBASEPOSEDERIVATIVE computes the base position and orientation
    %                           time derivative in terms of linear velocity
    %                           + quaternion derivative.

    %% --- Initialization ---

    % angular velocity in body coordinates
    w_omega_base   = nu_base_ikin(4:end);
    w_R_base       = wbc.rotationFromQuaternion(pose_base(4:7));
    b_omega_base   = transpose(w_R_base)*w_omega_base;

    % calculate the base quaternion derivative
    quat_base      = pose_base(4:end);
    dquat_base     = wbc.quaternionDerivative(quat_base,b_omega_base,1);

    % compute the base pose derivative
    pose_base_dot  = [nu_base_ikin(1:3); dquat_base];
end
