function v_base = computeBaseVelocitySeesaw(J_fixedLink,w_R_s,dqj,s_omega,seesaw,reg)

    % Velocity of the seesaw expressed in world frame
    w_omega = w_R_s * s_omega;

    % distance between the contact with the ground and the seesaw CoM w.r.t
    % world frame
    e3 = [0;0;1];
    w_r = (seesaw.delta*w_R_s*e3) -seesaw.rho*e3;

    % s_sFixed = positionOfFixedLink - seesawCoM  w.r.t. seesaw frame
    s_sFixed = seesaw.s_sFixed;

    % positionOfFixedLink - seesawCoM  w.r.t. world frame
    w_sFixed = w_R_s * s_sFixed;

    % linear and angular velocity of fixedLink in the world coordinates
    w_v_fixedLink = [skew(w_r - w_sFixed)*w_omega; 
                                           w_omega];
                                     
    % finally, base velocity in world coodinates
    invJb = pinv(J_fixedLink(1:6,1:6), reg.pinvTolVb);
    v_base = invJb*(w_v_fixedLink - J_fixedLink(1:6,7:end)*dqj);
    
end