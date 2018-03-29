function w_H_fixedLink = computeTransFromFixedLinkToWorld(w_R_s,seesaw)
    
    % Angles associated to w_R_s
    seesaw_rpy = rollPitchYawFromRotation(w_R_s);
 
    % s_sFixed = positionOfFixedLink - seesawCoM  w.r.t. seesaw frame
    s_sFixed = seesaw.s_sFixed;

    % OC = centerOfRotationSeesaw - originOfWorld  w.r.t world frame
    OC = [ 0;
          -seesaw.rho * seesaw_rpy(1);
           seesaw.rho];

    % CL = positionOfFixedLink - centerOfRotationSeesaw w.r.t. world frame
    e3 = [0;0;1];
    CL = w_R_s * (s_sFixed - seesaw.delta*e3);   

    % P_l = positionOfFixedLink w.r.t. world frame
    P_l = OC + CL;
    
    % Pose of the fixed link w.r.t. world frame at each time t
    w_H_fixedLink = [w_R_s,       P_l;
                     zeros(1,3),  1];              
end
