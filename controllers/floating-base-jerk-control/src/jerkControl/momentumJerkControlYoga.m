function [jointTorques_star, LDot_tilde, L_tilde, intL_tilde, xiDes, f_des, xiDot_star] = momentumJerkControlYoga(intLTilde_angular, nu, J_CoM, JL, JR, xCoM, w_H_l_sole, w_H_r_sole, L, M, h, JDotL_nu, JDotR_nu, x_dx_ddx_dddx_CoM_des, ...
                                                                                                                  gainsPCOM, gainsDCOM, LEFT_RIGHT_FOOT_IN_CONTACT, qj, qj_dqj_ddqj_des, impedances, dampings, ...
                                                                                                                  contactForces_est, xi_star, xi, contactForces_star, Config, Gain, Reg)
                               
    % MOMENTUMJERKCONTROL implements a momentum-based jerk controller.
    %
    % REFERENCES: IEEE-RAL 2018, "Momentum Control of an Underactuated Flying Humanoid Robot"; 
    %

    %% ------------Initialization----------------
    
    % parameters
    ndof            = size(M(7:end,7:end),1);
    
    % ROBUSTNESS w.r.t. modeling errors (inertial parameters)
    if Config.testRobustness
        
        M           = M * Config.robustnessFactor;
        h           = h * Config.robustnessFactor;
    end
    
    % get the robot total mass, and gravity forces, and feet positions
    m               = M(1,1);
    f_grav          = m*Config.GRAVITY_ACC*[0; 0; 1; zeros(3,1)];
    posLeftFoot     = w_H_l_sole(1:3,4); 
    posRightFoot    = w_H_r_sole(1:3,4);
        
    % compute momentum references
    [LDDot_des, LDot_des, L_des, intL_des] = computeMomentumReferences(x_dx_ddx_dddx_CoM_des, m);
    
    %% %%%%%%%%%%%%%%% COMPUTE THE MOMENTUM ACCELERATION %%%%%%%%%%%%%%% %%
    %
    % WHILE BALANCING: the momentum acceleration is defined as follows:
    %
    %   LDDot = Ac * Beta* xiDot + AcDot * contactForces. (1)
    %
    % The contact forces derivative have been parametrized as follows:
    %
    %   contactForcesDot = Beta * xiDot
    %
    % xiDot is then used as control input. Beta is an invertible matrix. 
    % It is therefore necessary to compute AcDot and Ac.
       
    % compute matrix Ac for both feet
    r_left          = posLeftFoot  - xCoM;
    r_right         = posRightFoot - xCoM;
    
    Ac_leftFoot     = [eye(3),           zeros(3);
                       wbc.skew(r_left), eye(3)];
                   
    Ac_rightFoot    = [eye(3),            zeros(3);
                       wbc.skew(r_right), eye(3)];
    
    % compute matrix Beta_left and Beta_right
    Beta_left       = computeParametrizationGradient(xi(1:6),   Config);
    Beta_right      = computeParametrizationGradient(xi(7:end), Config);
    
    % multiplier of contact forces derivatives
    Ac              = [Ac_leftFoot .* LEFT_RIGHT_FOOT_IN_CONTACT(1), ...
                       Ac_rightFoot .* LEFT_RIGHT_FOOT_IN_CONTACT(2)];
    Ac_Beta         = [Ac_leftFoot * Beta_left .* LEFT_RIGHT_FOOT_IN_CONTACT(1), ...
                       Ac_rightFoot * Beta_right .* LEFT_RIGHT_FOOT_IN_CONTACT(2)];
    
    % compute the time derivative of r_left and r_right
    nu_leftFoot     = JL * nu;
    nu_rightFoot    = JR * nu;
    v_CoM           = J_CoM(1:3,:) * nu;
    rDot_left       = nu_leftFoot(1:3)  - v_CoM;
    rDot_right      = nu_rightFoot(1:3) - v_CoM;
    
    % compute AcDot
    AcDot_leftFoot  = [zeros(3),             zeros(3);
                       wbc.skew(rDot_left),  zeros(3)]; 
                   
    AcDot_rightFoot = [zeros(3),             zeros(3);
                       wbc.skew(rDot_right), zeros(3)];
    
    AcDot           = [AcDot_leftFoot .* LEFT_RIGHT_FOOT_IN_CONTACT(1), ...
                       AcDot_rightFoot .* LEFT_RIGHT_FOOT_IN_CONTACT(2)];
    
    %% %%%%%%%%%%%%%%% MOMENTUM CONTROLLER DEFINITION  %%%%%%%%%%%%%%%%% %%
    %
    % The control input of Eq. (1) is: 
    %
    %  u1 = xiDot
    %
    % For the purpose of this documentation, it suffices to know that the
    % robot momentum converges to the desired values if the following 
    % equation is verified:
    %
    %  ATilde * u1 + deltaTilde = 0 (2)
    %
    % where: ATilde and deltaTilde must be properly calculated.       
    % For details on the controller implementation, see the RAL 2018 paper
    % at: https://ieeexplore.ieee.org/document/7997895. 
    
    % Compute the momentum error derivative/integral
    LDot_estimated  = Ac * [contactForces_est(1:6) .* LEFT_RIGHT_FOOT_IN_CONTACT(1); ...
                            contactForces_est(7:end) .* LEFT_RIGHT_FOOT_IN_CONTACT(2)] - f_grav;                   
    LDot_tilde      = LDot_estimated - LDot_des;
    L_tilde         = L - L_des;
    intL_tilde      = [(m * xCoM - intL_des(1:3)); intLTilde_angular];

    % Get the gains matrices
    KP_momentum     = blkdiag(diag(gainsPCOM), Gain.KP_AngularMomentum .*eye(3));
    KD_momentum     = blkdiag(diag(gainsDCOM), Gain.KD_AngularMomentum .*eye(3));
    
    % CONTROL LAW: momentum-based control with Lyapunov stability (IEEE-RAL 2018)
    KTilde          = KP_momentum + inv(Gain.KO_momentum) + KD_momentum;    
    deltaTilde      = KTilde * L_tilde + (KD_momentum + eye(6)) * LDot_tilde + ...
                      KP_momentum * intL_tilde - LDDot_des + AcDot * contactForces_est;

    % Primary task control input
    pinvAc_Beta       = [wbc.pinvDamped(Ac_leftFoot * Beta_left,1e-5) .* (1-LEFT_RIGHT_FOOT_IN_CONTACT(2)); zeros(6)] + ...
                        [zeros(6); wbc.pinvDamped(Ac_rightFoot * Beta_right,1e-5) .* (1-LEFT_RIGHT_FOOT_IN_CONTACT(1))] + ...
                         wbc.pinvDamped(Ac_Beta,1e-5).* LEFT_RIGHT_FOOT_IN_CONTACT(1).* LEFT_RIGHT_FOOT_IN_CONTACT(2);
                    
    xiDot_primaryTask = -pinvAc_Beta * deltaTilde;
    
    % Null space projector (forces)
    Null_Ac_Beta      = (eye(12) - pinvAc_Beta*Ac_Beta)*LEFT_RIGHT_FOOT_IN_CONTACT(1)*LEFT_RIGHT_FOOT_IN_CONTACT(2);    
    
    %% %%%%%%%%%%%%%%%%% TORQUE CONTROLLER DEFINITION  %%%%%%%%%%%%%%%%% %%
    
    % Get the required matrices
    Jc                = [JL .*LEFT_RIGHT_FOOT_IN_CONTACT(1); JR .*LEFT_RIGHT_FOOT_IN_CONTACT(2)];
    JcDot_nu          = [JDotL_nu .*LEFT_RIGHT_FOOT_IN_CONTACT(1); JDotR_nu .*LEFT_RIGHT_FOOT_IN_CONTACT(2)];
    B                 = [zeros(6,ndof); eye(ndof)];
    Lambda            = Jc/M*B;
    pinvLambda        = wbc.pinvDamped(Lambda, Reg.pinvDamp); 
    NLambda           = eye(ndof) - pinvLambda*Lambda;
    
    % terms required for calculating u_0
    qjDot             = nu(7:end);
    KP_torqueControl  = diag(impedances);
    KD_torqueControl  = diag(dampings);
    jointPos_err      = qj    - qj_dqj_ddqj_des(:,1);
    jointVel_err      = qjDot - qj_dqj_ddqj_des(:,2);
    
    % consitency with the current gain tuning
    USE_MBAR          = false;
    USE_ACC           = 0;
    
    if USE_MBAR
        
        MBar          = (M(7:end,7:end) - M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end)); %#ok<UNRCH>
    else
        MBar          = eye(ndof);
    end
    
    u_0               = MBar*(USE_ACC*qj_dqj_ddqj_des(:,3) - KD_torqueControl * jointVel_err - KP_torqueControl * jointPos_err); 
    
    % terms required for calculating tau_0
    tau_0             = h(7:end) -M(7:end,1:6)/M(1:6,1:6)*h(1:6) ...
                       -transpose(Jc(:,7:end))*contactForces_est + ...
                        M(7:end,1:6)/M(1:6,1:6)*transpose(Jc(:,1:6))*contactForces_est + u_0;
      
    jointTorques_star = pinvLambda*(Jc/M*(h-Jc'*contactForces_star) -JcDot_nu) + NLambda*tau_0;
    
    %% %%%%%%%%%%%%%%%%%%%%% NULL SPACE OF MOMENTUM %%%%%%%%%%%%%%%%%%%% %%
    
    % Now, rewrite the joint torques equations as:
    %
    %    tau = Sigma * f_star + gamma_full
    %
    % where we isolated the terms multiplied by the forces in the
    % joint torques equations. Finally we compute the time derivative of tau:
    %
    %    tauDot = SigmaDot * f_star + Sigma * fDot_star + gamma_fullDot
    %    tauDot ~ Sigma * fDot_star (approximated) + gamma_redDot
    %
    % Then, consider the following Lyapunov function candidate:
    %
    %    V    = 1/2*|tau|^2
    %    VDot = tau^T*tauDot
    %
    % It is clear that choosing tauDot = -K_tau*tau will ensures the
    % convergence of the joint torques to zero.
    %
    
    % theoretically, the forces inside tau_0 are the measured ones, not the
    % desired. Therefore only the derivative of the desired forces should
    % be used to minimize the joint torques, however I am not really sure
    % about this.
    DEACTIVATE_NULL_SPACE_FORCES = 0;
    
    Sigma               = -pinvLambda*(Jc/M*Jc') -DEACTIVATE_NULL_SPACE_FORCES.*NLambda*(transpose(Jc(:,7:end)) - M(7:end,1:6)/M(1:6,1:6)*transpose(Jc(:,1:6)));
    Sigma_Beta          =  Sigma * Null_Ac_Beta;
    gamma_redDot        =  Sigma * xiDot_primaryTask -Gain.useVelFeedbackForMinTorques*NLambda*MBar*KP_torqueControl*jointVel_err;   
    K_tau               =  Gain.K_tau;
  
    % Secondary task   
    pinvSigma_Beta      =  wbc.pinvDamped(Sigma_Beta, Reg.pinvDamp);
    xiDot_secondaryTask = -pinvSigma_Beta*(K_tau*jointTorques_star + gamma_redDot);
  
    % FINAL control input (Parametrized)
    pinvAc              = [wbc.pinvDamped(Ac_leftFoot,1e-5) .* (1-LEFT_RIGHT_FOOT_IN_CONTACT(2)); zeros(6)] + ...
                          [zeros(6); wbc.pinvDamped(Ac_rightFoot,1e-5) .* (1-LEFT_RIGHT_FOOT_IN_CONTACT(1))] + ...
                           wbc.pinvDamped(Ac,1e-5).* LEFT_RIGHT_FOOT_IN_CONTACT(1).* LEFT_RIGHT_FOOT_IN_CONTACT(2);
 
    f_des               = pinvAc*(LDot_des + f_grav);                   
    
    if LEFT_RIGHT_FOOT_IN_CONTACT(1) > 0.5
        
        xi_des_left = fromForcesToParametrization(f_des(1:6),Config);
    else
        xi_des_left = zeros(6,1);
    end
    if LEFT_RIGHT_FOOT_IN_CONTACT(2) > 0.5
        
        xi_des_right = fromForcesToParametrization(f_des(7:end),Config);
    else
        xi_des_right = zeros(6,1);
    end
    
    xiDes               = [xi_des_left;xi_des_right];
    Ke                  = Gain.Ke;
    xiDot_star          = xiDot_primaryTask + Null_Ac_Beta*xiDot_secondaryTask -Ke*(xi_star-[xiDes(1:6)*LEFT_RIGHT_FOOT_IN_CONTACT(1);xiDes(7:end)*LEFT_RIGHT_FOOT_IN_CONTACT(2)]);
end