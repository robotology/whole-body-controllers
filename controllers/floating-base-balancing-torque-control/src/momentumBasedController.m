function [HessianMatrixOneFoot, gradientOneFoot, ConstraintsMatrixOneFoot, bVectorConstraintsOneFoot, ...
          HessianMatrixTwoFeet, gradientTwoFeet, ConstraintsMatrixTwoFeet, bVectorConstraintsTwoFeet, ...
          tauModel, Sigma, Na, f_LDot] =  ...
              momentumBasedController(feetContactStatus, ConstraintsMatrix, bVectorConstraints, jointPos, jointPos_des, nu, M, h, L, intL_angMomError, w_H_l_sole, w_H_r_sole, ...
                                      J_l_sole, J_r_sole, JDot_l_sole_nu, JDot_r_sole_nu, pos_CoM, J_CoM, desired_pos_vel_acc_CoM, KP_CoM, KD_CoM, KP_postural, Config, Reg, Gain)
    
    % MOMENTUMBASEDCONTROLLER implements a momentum-based whole body
    %                         balancing controller for humanoid robots.
    %
    % REFERENCES: G. Nava and F. Romano and F. Nori and D. Pucci, 
    %            "Stability Analysis and Design of Momentum Based Controllers for Humanoid Robots",
    %             Available at: https://ieeexplore.ieee.org/document/7759126/

    %% --- Initialization ---

    % Compute the momentum rate of change. The momentum rate of change
    % equals the summation of the external forces and moments, i.e.:
    %
    %    LDot = A*f + f_grav (1)
    %
    % where A is the matrix mapping the forces and moments into the
    % momentum equations, f_grav is the gravity force, f is a vector stacking
    % all the external forces and moments acting on the robot as follows:
    %
    %    f = [f_left; f_right]
    %
    % where f_left are the forces and moments acting on the left foot and
    % f_right are the forces and moments acting on the right foot.
    
    % Compute the gravity force
    m             = M(1,1);
    gravAcc       = Config.GRAV_ACC;
    f_grav        = [zeros(2,1);
                    -m*gravAcc;
                     zeros(3,1)];
    
    % Compute matrix A in Eq. (1)
    pos_leftFoot  = w_H_l_sole(1:3,4);
    pos_rightFoot = w_H_r_sole(1:3,4);

    % Distance between the application points of the contact forces w.r.t. CoM
    r_left        = pos_leftFoot  - pos_CoM; 
    r_right       = pos_rightFoot - pos_CoM; 
    
    % Partition matrix A into the part that multiplies the left foot
    % wrenches and the right foot wrenches, i.e. A = [A_left, A_right]
    A_left        = [eye(3),           zeros(3);
                     wbc.skew(r_left), eye(3)];
    A_right       = [eye(3),            zeros(3);
                     wbc.skew(r_right), eye(3)];

    A             = [A_left, A_right]; 
    
    %% MOMENTUM CONTROL
    %
    % We would like to achieve a desired momentum's dynamics:
    %
    %    LDot_star = LDot_des - KP_momentum*(L-LDes) - KI_momentum*(intL-intLDes)
    %
    % where intL is the integral of the momentum. Assume the contact forces
    % and moments can be considered as control inputs of Eq. (1). Then, the
    % problem is to find f such that:
    %
    %    LDot_star = A*f + f_grav (2)
    %
    % We must now distinguish two different cases:
    %
    % CASE 1: the robot is balancing on one foot. In this case, the solution 
    %         to Eq. (2) is:
    %
    %    f = A^(-1)*(LDot_star - f_grav) (3)
    %
    % CASE 2: the robot is balancing on two feet. In this case, there is
    %         redundancy as there are more control inputs (12) than variables 
    %         to control (6). Therefore one can write:
    %
    %    f = pinvA*(LDot_star - f_grav) + Na*f_0 (4)
    %
    % where pinvA is the pseudoinverse of matrix A and Na is its null space
    % projector. f_0 is a free variable that does not affect the momentum
    % dynamics Eq (1).
    
    % Gains mapping. 
    %
    %    KP_momentum = blkdiag(KD_CoM, KP_angMom)
    %    KD_momentum = blkdiag(KP_CoM, KI_angMom)
    %
    KP_angMom    = Gain.KP_AngularMomentum*eye(3);
    KI_angMom    = Gain.KI_AngularMomentum*eye(3);
    
    % Desired CoM dynamics (conseguently, linear momentum)
    vel_CoM      = J_CoM(1:3,:) * nu;
    acc_CoM_star = desired_pos_vel_acc_CoM(:,3) - KP_CoM*(pos_CoM - desired_pos_vel_acc_CoM(:,1)) - KD_CoM*(vel_CoM - desired_pos_vel_acc_CoM(:,2));

    % Desired momentum dynamics
    LDot_star    = [m * acc_CoM_star;
                   (-KP_angMom * L(4:end) -KI_angMom * intL_angMomError)];
       
    %% CASE 1: one foot balancing
    %
    % In this case, we make use of a QP solver. In particular, Eq. (3) is rewritten as:
    %
    %    f^T*A^T*A*f - f^T*A^T*(LDot_star - f_grav) = 0 (5)
    %
    % that is the quadratic problem associated with Eq. (3). Now rewrite
    % Eq. (5) as:
    %
    %    f^T*Hessian*f + f^T*gradient = 0
    %
    % where Hessian = A^T*A and gradient = - A^T*(LDot_star - f_grav). Now
    % it is possible to solve the folowing QP problem:
    %
    % f_star = argmin_f |f^T*Hessian*f + f^T*gradient|^2
    %
    %          s.t. C*f < b
    %
    % where the inequality constraints represent the unilateral, friction
    % cone and local CoP constraints at the foot.
    
    % Hessian matrix and gradient QP one foot
    A_oneFoot                 =  A_left*feetContactStatus(1)*(1 - feetContactStatus(2)) + A_right*feetContactStatus(2)*(1 - feetContactStatus(1));
    HessianMatrixOneFoot      =  A_oneFoot'*A_oneFoot + eye(size(A_oneFoot,2))*Reg.HessianQP;
    gradientOneFoot           = -A_oneFoot'*(LDot_star - f_grav);
    
    % Update constraint matrices. The contact wrench associated with the 
    % left foot (resp. right foot) is subject to the following constraint:
    %
    %     ConstraintMatrixLeftFoot * l_sole_f_left < bVectorConstraints
    %
    % In this case, however, f_left is expressed w.r.t. the frame l_sole,
    % which is solidal to the left foot. The contact forces f used in the
    % controller however are expressed w.r.t. the frame l_sole[w], that is
    % a frame with the origin at the contact location but the orientation
    % of the inertial frame. For this reason, the mapping between the two
    % frames is given by:
    %
    %    l_sole_f_left = blkdiag(l_sole_R_w, l_sole_R_w) * l_sole[w]_f_left
    %
    % therefore we rewrite the contact constraints as:
    %
    %    ConstraintMatrixLeftFoot * blkdiag(l_sole_R_w, l_sole_R_w) * l_sole[w]_f_left < bVectorConstraints
    %
    % and this in the end results in updating the constraint matrix as follows:
    %
    %    ConstraintMatrixLeftFoot = ConstraintMatrixLeftFoot * blkdiag(l_sole_R_w, l_sole_R_w) 
    %
    % The same holds for the right foot.
    %
    w_R_r_sole                = w_H_r_sole(1:3,1:3);
    w_R_l_sole                = w_H_l_sole(1:3,1:3);
    ConstraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(w_R_l_sole', w_R_l_sole');
    ConstraintMatrixRightFoot = ConstraintsMatrix * blkdiag(w_R_r_sole', w_R_r_sole');
    
    % One foot constraints
    ConstraintsMatrixOneFoot  = feetContactStatus(1) * (1 - feetContactStatus(2)) * ConstraintMatrixLeftFoot + ...
                                feetContactStatus(2) * (1 - feetContactStatus(1)) * ConstraintMatrixRightFoot;
    bVectorConstraintsOneFoot = bVectorConstraints;

    %% CASE 2: two feet balancing
    %
    % In this case, we solve Eq (4) by means of the matrix pseudoinverse and 
    % NOT through the QP. The QP is instead used to calculate the vector 
    % projected in the null space (f_0). In particular, we choose f_0 in
    % order to minimize the joint torques magnitude. To do so, it is necessary
    % to write down the relationship between the joint torques and the
    % contact forces:
    %
    %    tau = pinvLambda*(Jc*invM*(h - Jc^T*f) -JcDot_nu) + NLambda*tau_0 (6)
    %
    % where tau_0 is given by the following equation:
    %
    %    tau_0 = hs - Msb*invMb*hb - (Js^T - Msb*invMb*Jb^T)*f + u_0
    %
    % where we have:
    %
    %    M = [Mb, Mbs;    h = [hb;    Jc = [Jb, Js]
    %         Msb, Ms];        hs];
    %    
    % obtained by partitioning the dynamics in order to split the first
    % six rows and the remaining NDOF rows.
    %
    % u_0 instead are the feedback terms associated with the postural task,
    % and therefore are given by the following expression (for more info,
    % look at the reference paper):
    %
    %    u_0 = -KP_postural*NLambda*Mbar*jointPosTilde -KD_postural*NLambda*Mbar*jointVel
    %         
    % where Mbar =  Ms-Msb/Mb*Mbs.
    %
    % Now, let us rewrite Eq. (6) in order to isolate the terms which
    % depend on the contact forces:
    %
    %    tau = Sigma*f + tauModel  (7)   
    %
    % where Sigma    = -(pinvLambda*Jc*invM*Jc^T + NLambda*(Js^T - Msb*invMb*Jb^T))
    %
    %       tauModel = pinvLambda*(Jc*invM*h -JcDot_nu) + ...
    %                  NLambda*(hs - Msb*invMb*hb + u_0)
    %
    % Finally, we substitute Eq. (4) into Eq. (7) which gives:
    %
    %    tau = Sigma*pinvA*(LDot_star - f_grav) + Sigma*Na*f_0 + tauModel (8)
    %
    % minimizing the torques implies we would like to have tau = 0 in Eq.
    % (8) (note that it is not possible to achieve tau = 0 by choosing f_0)
    % 
    % It is possible to write down Eq. (8) as a QP problem, as we
    % did for Eq. (5):
    %
    %    f_0^T*Hessian*f_0 + f_0^T*gradient = 0 (9)
    % 
    % where Hessian  = transpose(Sigma*Na)*Sigma*Na 
    %       gradient = transpose(Sigma*Na)*(Sigma*pinvA*(LDot_star - f_grav) + tauModel)
    %
    % The associated QP formulation is now:
    %
    % f_0_star = argmin_f_0 |f_0^T*Hessian*f_0 + f_0^T*gradient|^2
    %
    %            s.t. C*f_0 < b
    %
    % Note that in this way we are assuming that the part of the contact
    % forces dedicated to stabilize the momentum dynamics, i.e. the term
    %
    %    f_LDot = pinvA*(LDot_star - f_grav)
    %
    % is does not violate the constraints.
    
    % Compute f_LDot 
    pinvA       = pinv(A, Reg.pinvTol); 
    f_LDot      = pinvA*(LDot_star - f_grav);
                
    % Null space of the matrix A            
    Na          = (eye(12,12) - pinvA*A).*feetContactStatus(1).*feetContactStatus(2);
    
    %% Compute Sigma and tauModel
    %
    % NOTE that the formula Eq (7) will be used for computing the torques 
    % also in case  the robot is balancing on ONE foot. In fact, in that
    % case, f will be a vector of the form (left foot balancing):
    %
    %    f = [f_left (from QP); zeros(6,1)];
    %
    % same holds for the right foot balancing. The additional zeros are
    % required in order to match the dimension of Sigma (NDOF x 12).
    
    % Contact jacobians
    NDOF        = size(J_l_sole(:,7:end),2);
    Jc          = [J_l_sole.*feetContactStatus(1);      
                   J_r_sole.*feetContactStatus(2)];
                   
    % Jacobian derivative dot(Jc)*nu
    JcDot_nu    = [JDot_l_sole_nu.*feetContactStatus(1);      
                   JDot_r_sole_nu.*feetContactStatus(2)];

    % Selector of actuated DoFs
    B           = [zeros(6,NDOF);
                   eye(NDOF,NDOF)];
                            
    % The mass matrix is partitioned as:
    %
    %   M = [ Mb,   Mbs
    %         Mbs', Ms ];  
    %
    % where: Mb  \in R^{6 x 6}
    %        Mbs \in R^{6 x 6+NDOF}
    %        Ms  \in R^{NDOF x NDOF}
    %
    Mb          = M(1:6,1:6);
    Mbs         = M(1:6,7:end);
    Ms          = M(7:end,7:end);
                 
    % Get matrix Sigma        
    Jc_invM     =  Jc/M;
    Lambda      =  Jc_invM*B;
    pinvLambda  =  wbc.pinvDamped(Lambda, Reg.pinvDamp); 
    NullLambda  =  eye(NDOF) - pinvLambda*Lambda;
    Sigma       = -(pinvLambda*Jc_invM*Jc' + NullLambda*(transpose(Jc(:,7:end)) -Mbs'/Mb*transpose(Jc(:,1:6))));
    
    % Mbar is the mass matrix associated with the joint dynamics, i.e.
    Mbar            = Ms-Mbs'/Mb*Mbs;
    NullLambda_Mbar = NullLambda*Mbar;
    
    % Adaptation of the control gains for back compatibility with the older
    % versions of the controller
    KP_postural     = KP_postural*pinv(NullLambda_Mbar, Reg.pinvTol) + Reg.KP_postural*eye(NDOF);
    KD_postural     = diag(Gain.KD_postural)*pinv(NullLambda_Mbar,Reg.pinvTol) + Reg.KD_postural*eye(NDOF);
    
    % Joints velocity and joints position error
    jointVel        = nu(7:end);
    jointPosTilde   = jointPos - jointPos_des;
    
    % Get the vector tauModel
    u_0             = -KP_postural*NullLambda_Mbar*jointPosTilde -KD_postural*NullLambda_Mbar*jointVel;
    tauModel        =  pinvLambda*(Jc_invM*h - JcDot_nu) + NullLambda*(h(7:end) - Mbs'/Mb*h(1:6) + u_0);
  
    %% QP parameters for two feet standing
    %
    % In the case the robot stands on two feet, the control objective is 
    % the minimization of the joint torques through the redundancy of the 
    % contact forces. See Previous comments.

    % Get the inequality constraints matrices
    ConstraintsMatrixBothFeet  = blkdiag(ConstraintMatrixLeftFoot,ConstraintMatrixRightFoot);
    bVectorConstraintsBothFeet = [bVectorConstraints;bVectorConstraints];
    
    % The optimization problem Eq. (9) seeks for the redundancy of the external
    % wrench that minimize joint torques. Recall that the contact wrench can 
    % be written as:
    %
    %     f = f_LDot + Na*f_0 
    %
    % Then, the constraints on the contact wrench is of the form
    %
    %     ConstraintsMatrixBothFeet*f < bVectorConstraints
    %
    % which in terms of f0 is:
    %
    % ConstraintsMatrixBothFeet*Na*f0 < bVectorConstraints - ConstraintsMatrixBothFeet*f_LDot
    %
    ConstraintsMatrixTwoFeet  = ConstraintsMatrixBothFeet*Na;
    bVectorConstraintsTwoFeet = bVectorConstraintsBothFeet - ConstraintsMatrixBothFeet*f_LDot;
    
    % Evaluation of Hessian matrix and gradient vector for solving the
    % optimization problem Eq. (9)
    Sigma_Na                  = Sigma*Na;
    HessianMatrixTwoFeet      = Sigma_Na'*Sigma_Na + eye(size(Sigma_Na,2))*Reg.HessianQP;
    gradientTwoFeet           = Sigma_Na'*(tauModel + Sigma*f_LDot);
end
