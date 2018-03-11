%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function  [tauModel, Sigma, NA, f_HDot, ...
          HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot, bVectorConstraintsQp1Foot, ...
          HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet, bVectorConstraintsQp2Feet, ...
          errorCoM, f_noQP, HError, HDotDes, HDotDes_test,alpha,JcMinvJct] = ...
              balancingControllerYogaFriction(constraints, ROBOT_DOF_FOR_SIMULINK, ConstraintsMatrix, bVectorConstraints, ...
                  qj, qjDes, nu, M, h, H, intHw, w_H_l_sole, w_H_r_sole, JL, JR, dJL_nu, dJR_nu, xCoM, J_CoM, desired_x_dx_ddx_CoM, ...
                  gainsPCOM, gainsDCOM, impedances, qjDesDot, JG, Gain, Reg)
              
    %BALANCING CONTROLLER

    %% DEFINITION OF CONTROL AND DYNAMIC VARIABLES
    pos_leftFoot   = w_H_l_sole(1:3,4);
    w_R_l_sole     = w_H_l_sole(1:3,1:3);
    pos_rightFoot  = w_H_r_sole(1:3,4);
    w_R_r_sole     = w_H_r_sole(1:3,1:3);

    dampings       = Gain.dampings;
    ROBOT_DOF      = size(ROBOT_DOF_FOR_SIMULINK,1);
    gravAcc        = 9.81;
    
    % Mass of the robot
    m              = M(1,1);
    
    % The mass matrix is partitioned as:
    %
    %   M = [ Mb,   Mbj
    %         Mbj', Mj ];  
    %
    % where: Mb  \in R^{6x6}
    %        Mbj \in R^{6x6+nDof}
    %        Mj  \in R^{nDofxnDof}
    %
    Mb             = M(1:6,1:6);
    Mbj            = M(1:6,7:end);
    Mj             = M(7:end,7:end);

    St             = [zeros(6,ROBOT_DOF);
                      eye(ROBOT_DOF,ROBOT_DOF)];
    gravityWrench  = [zeros(2,1);
                     -m*gravAcc;
                      zeros(3,1)];

    % Velocity of the center of mass
    xCoM_dot       = J_CoM(1:3,:)*nu;
    
    % Joint velocity
    qjDot          = nu(7:end);
    
    % Joint position error
    qjTilde        =  qj-qjDes(1:ROBOT_DOF);
    
    % Desired acceleration for the center of mass
    xDDcomStar     = desired_x_dx_ddx_CoM(:,3) -gainsPCOM.*(xCoM - desired_x_dx_ddx_CoM(:,1)) -gainsDCOM.*(xCoM_dot - desired_x_dx_ddx_CoM(:,2));
   
    % Application point of the contact force on the right foot w.r.t. CoM
    Pr              = pos_rightFoot -xCoM; 
    
    % Application point of the contact force on the left foot w.r.t. CoM
    Pl              = pos_leftFoot  -xCoM; 

    % The following variables serve for determening the rate-of-change of
    % the robot's momentum. In particular, when balancing on two feet, one has:
    %
    %   dot(H) = gravityWrench +  AL*f_L + AR*f_R
    %          = gravityWrench + [AL,AR]*f
    %
    % where  f_L and f_R are the contact wrenches acting on the left and
    % right foot, respectively, and f = [f_L;f_R].
    %
    AL              = [ eye(3),zeros(3);
                        skew(Pl), eye(3)];
    AR              = [ eye(3), zeros(3);
                        skew(Pr), eye(3)];

    % dot(H) = mg + A*f
    A               = [AL, AR]; 
    
    pinvA           = pinv(A, Reg.pinvTol)*constraints(1)*constraints(2)  ...
                      + [inv(AL);zeros(6)]*constraints(1)*(1-constraints(2)) ... 
                      + [zeros(6);inv(AR)]*constraints(2)*(1-constraints(1)); 
                
    % Null space of the matrix A            
    NA              = (eye(12,12)-pinvA*A)*constraints(1)*constraints(2);

    % Time varying contact jacobian
    Jc              = [JL*constraints(1);      
                       JR*constraints(2)];
                   
    % Time varying dot(J)*nu
    Jc_nuDot        = [dJL_nu*constraints(1) ;      
                       dJR_nu*constraints(2)];

    JcMinv          = Jc/M;
    JcMinvSt        = JcMinv*St;
    JcMinvJct       = JcMinv*transpose(Jc);
    
    % multiplier of f in tau
    JBar            = transpose(Jc(:,7:end)) -Mbj'/Mb*transpose(Jc(:,1:6)); 

    Pinv_JcMinvSt   = pinvDamped(JcMinvSt,Reg.pinvDamp); 
   
    % nullJcMinvSt --> null space of Pinv_JcMinvSt
    nullJcMinvSt    = eye(ROBOT_DOF) - Pinv_JcMinvSt*JcMinvSt;

    % Mbar is the mass matrix associated with the joint dynamics, i.e.
    Mbar            = Mj-Mbj'/Mb*Mbj;
    NLMbar          = nullJcMinvSt*Mbar;
    
    % Adaptation of control gains for back compatibility with older
    % versions of the controller
    impedances      = diag(impedances)*pinv(NLMbar,Reg.pinvTol) + Reg.impedances*eye(ROBOT_DOF);
    dampings        = diag(dampings)*pinv(NLMbar,Reg.pinvTol)   + Reg.dampings*eye(ROBOT_DOF); 
  
    %% QP PARAMETERS FOR TWO FEET STANDING
    % In the case the robot stands on two feet, the control objective is 
    % the minimization of the joint torques through the redundancy of the 
    % contact forces. By direct calculations one shows that the joint
    % torqes take the following form:
    %
    % 0) tau = tauModel + Sigma*f_HDot + SigmaNA*f0
    %
    % where f0 is the redundancy of the contact wrenches. Then, the problem
    % is defined as follows:
    %
    % 1) f0  = argmin |tau(f0)|^2
    %          s.t.
    %          ConstraintsMatrixQP2Feet*f0 < bVectorConstraintsQp2Feet
    
    % Update constraint matrices. The constraint matrix for the inequality
    % constraints in the problem 1) is built up startin from the constraint
    % matrix associated with each single foot. More precisely, the contact
    % wrench associated with the left foot (resp. right foot) is subject to
    % the following constraint:
    %
    % constraintMatrixLeftFoot*l_sole_f_L < bVectorConstraints
    %
    % In this case, however, f_L is expressed w.r.t. the frame l_sole,
    % which is solidal to the left foot. Since the controller uses contact
    % wrenches expressed w.r.t. a frame whose orientation is that of the
    % inertial frame, we have to update the constraint matrix according to
    % the transformation w_R_l_sole, i.e.
    %
    % constraintMatrixLeftFoot = ConstraintsMatrix*w_R_l_sole
    %
    % The same hold for the right foot
    %
    constraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(w_R_l_sole',w_R_l_sole');
    constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(w_R_r_sole',w_R_r_sole');
    ConstraintsMatrix2Feet    = blkdiag(constraintMatrixLeftFoot,constraintMatrixRightFoot);
    bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];
    
    % Terms used in Eq. 0)
    tauModel  = Pinv_JcMinvSt*(JcMinv*h -Jc_nuDot) + nullJcMinvSt*(h(7:end) - Mbj'/Mb*h(1:6) ...
               -impedances*NLMbar*qjTilde -dampings*NLMbar*(qjDot-qjDesDot) +Gain.Kf*qjDesDot);
           
    Sigma     = -(Pinv_JcMinvSt*JcMinvJct + nullJcMinvSt*JBar);
    
    
     %% NEW TERMS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    HDes = [(m*(desired_x_dx_ddx_CoM(:,2))); zeros(3,1)];
    HError = H-HDes;
    
    % Time varying contact jacobian
 
    JcMinvR          = JR/M;
    JcMinvStR        = JcMinvR*St;
    JcMinvJctR       = JcMinvR*transpose(JR);
    
    % Compute multiplier of nu_b  
    pinvJb = (transpose(Jc(:,1:6))*Jc(:,1:6) + Reg.pinvDamp_nu_b*eye(6))\transpose(Jc(:,1:6));  
  
    % Reduced CMM
    JG_red = -JG(1:6,1:6)*pinvJb*Jc(:,7:end) +JG(1:6,7:end);
    
    if sum(constraints) > 1.5
            
        B1 = A*(pinvDamped(JcMinvJct,Reg.pinvDampB1))*JcMinvSt;
        B2 = A*(pinvDamped(JcMinvJct,Reg.pinvDampB2))*JcMinvSt;
        Null_friction = eye(ROBOT_DOF)+transpose(B2)*JG_red;
        
        
        alpha = transpose(HError)*B2*Gain.Kf*Null_friction*qjDot;
        
    else
        
        B1 = AR*(pinvDamped(JcMinvJctR,Reg.pinvDampB1))*JcMinvStR;
        B2 = AR*(pinvDamped(JcMinvJctR,Reg.pinvDampB2))*JcMinvStR;
        Null_friction = eye(ROBOT_DOF)+transpose(B2)*JG_red;
         
        alpha = transpose(HError)*B2*Gain.Kf*Null_friction*qjDot;
    end
    
    if alpha < -0.1
        
    % Desired rate-of-change of the robot momentum
    HDotDes   = [m*xDDcomStar ;
                -Gain.KD_AngularMomentum*H(4:end)-Gain.KP_AngularMomentum*intHw] +B1*Gain.Kf*transpose(B1)*HDes;
                 
    else
        
    % Desired rate-of-change of the robot momentum
    HDotDes   = [m*xDDcomStar ;
                -Gain.KD_AngularMomentum*H(4:end)-Gain.KP_AngularMomentum*intHw] +B1*Gain.Kf*transpose(B1)*HDes +0*B2*Gain.Kf*Null_friction*qjDot;
    end
    
    % Desired rate-of-change of the robot momentum
    HDotDes_test   = [m*xDDcomStar ;
                     -Gain.KD_AngularMomentum*H(4:end)-Gain.KP_AngularMomentum*intHw];

    % Contact wrenches realizing the desired rate-of-change of the robot
    % momentum HDotDes when standing on two feet. Note that f_HDot is
    % different from zero only when both foot are in contact, i.e. 
    % constraints(1) = constraints(2) = 1. This because when the robot
    % stands on one foot, the f_HDot is evaluated directly from the
    % optimizer (see next section).
    f_HDot    = pinvA*(HDotDes - gravityWrench)*constraints(1)*constraints(2);
    SigmaNA   = Sigma*NA;
   
    % The optimization problem 1) seeks for the redundancy of the external
    % wrench that minimize joint torques. Recall that the contact wrench can 
    % be written as:
    %
    % f = f_HDot + NA*f_0 
    %
    % Then, the constraints on the contact wrench is of the form
    %
    % ConstraintsMatrix2Feet*f < bVectorConstraints,
    %
    % which in terms of f0 is:
    %
    % ConstraintsMatrix2Feet*NA*f0 < bVectorConstraints - ConstraintsMatrix2Feet*f_HDot
    ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
    bVectorConstraintsQp2Feet = transpose(bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot);
    
    % Evaluation of Hessian matrix and gradient vector for solving the
    % optimization problem 1).
    HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*Reg.HessianQP;
    gradientQP2Feet           = transpose(SigmaNA'*(tauModel + Sigma*f_HDot));

    %% QP PARAMETERS FOR ONE FOOT STANDING
    % In the case the robot stands on one foot, there is no redundancy of
    % the contact wrenches. Hence, we cannot use this redundancy for
    % minimizing the joint torques. For this reason, the minimization
    % problem is modified as follows:
    %
    % 2) f = argmin|dot(H)(f) - dot(H)_des|^2
    %        s.t.
    %        ConstraintsMatrixQP1Foot*f < bVectorConstraintsQp1Foot
    %
    % where f is the contact wrench either of the left or on the right
    % foot.
    %
    ConstraintsMatrixQP1Foot  = constraints(1) * (1 - constraints(2)) * constraintMatrixLeftFoot + ...
                                constraints(2) * (1 - constraints(1)) * constraintMatrixRightFoot;
    bVectorConstraintsQp1Foot = transpose(bVectorConstraints);

    A1Foot                    =  AL*constraints(1)*(1-constraints(2)) + AR*constraints(2)*(1-constraints(1));
    HessianMatrixQP1Foot      =  A1Foot'*A1Foot + eye(size(A1Foot,2))*Reg.HessianQP;
    gradientQP1Foot           = transpose(-A1Foot'*(HDotDes - gravityWrench));

    %% DEBUG DIAGNOSTICS
    
    % Unconstrained solution for the problem 1)
    f0                        = -pinvDamped(SigmaNA, Reg.pinvDamp*1e-5)*(tauModel + Sigma*f_HDot);
    
    % Unconstrained contact wrenches
    f_noQP                    =  pinvA*(HDotDes - gravityWrench) + NA*f0*constraints(1)*constraints(2); 
    
    % Error on the center of mass
    errorCoM                  =  xCoM - desired_x_dx_ddx_CoM(:,1);
    
    
    %% NEW TERMS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    HDes = [(m*(desired_x_dx_ddx_CoM(:,2))); zeros(3,1)];
    HError = H-HDes;
    
    % Time varying contact jacobian
 
    JcMinvR          = JR/M;
    JcMinvStR        = JcMinvR*St;
    JcMinvJctR       = JcMinvR*transpose(JR);
    
    % Compute multiplier of nu_b  
    pinvJb = (transpose(Jc(:,1:6))*Jc(:,1:6) + Reg.pinvDamp_nu_b*eye(6))\transpose(Jc(:,1:6));  
  
    % Reduced CMM
    JG_red = -JG(1:6,1:6)*pinvJb*Jc(:,7:end) +JG(1:6,7:end);
    
    if sum(constraints) > 1.5
        
        
        B = A*(pinvDamped(JcMinvJct,Reg.pinvDamp))*JcMinvSt;
        Null_friction = eye(ROBOT_DOF)+transpose(B)*JG_red;
        
        
        alpha = transpose(HError)*B*Gain.Kf*Null_friction*qjDot;
        
    else
        
        B = AR*(pinvDamped(JcMinvJctR,Reg.pinvDamp))*JcMinvStR;
        Null_friction = eye(ROBOT_DOF)+transpose(B)*JG_red;
         
        alpha = transpose(HError)*B*Gain.Kf*Null_friction*qjDot;
    end
    
    if alpha < -0.1
        
    % Desired rate-of-change of the robot momentum
    HDotDes_test_old   = [m*xDDcomStar ;
                     -Gain.KD_AngularMomentum*H(4:end)-Gain.KP_AngularMomentum*intHw] +B*Gain.Kf*transpose(B)*HDes;
                 
    else
        
    % Desired rate-of-change of the robot momentum
    HDotDes_test_old   = [m*xDDcomStar ;
                     -Gain.KD_AngularMomentum*H(4:end)-Gain.KP_AngularMomentum*intHw] +B*Gain.Kf*transpose(B)*HDes -B*Gain.Kf*Null_friction*qjDot;
    end

end
