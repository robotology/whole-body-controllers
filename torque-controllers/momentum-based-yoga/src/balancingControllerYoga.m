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
function [tau_model, Sigma, Null_A, f_HDot_des, ...
          HessianMatrix_QP1Foot, gradient_QP1Foot, ConstraintsMatrix_QP1Foot, bVectorConstraints_Qp1Foot, ...
          HessianMatrix_QP2Feet, gradient_QP2Feet, ConstraintsMatrix_QP2Feet, bVectorConstraints_Qp2Feet, ...
          CoM_error] =  ...
              balancingControllerYoga(feet_in_contact, ROBOT_DOF_FOR_SIMULINK, ConstraintsMatrix, bVectorConstraints, ...
                                      s, s_des, nu, M, h, L, intL, w_H_l_sole, w_H_r_sole, J_l_sole, J_r_sole, JDot_l_sole_nu, JDot_r_sole_nu, ...
                                      x_CoM, J_CoM, desired_x_xDot_xDDot_CoM, Kp_CoM, Kd_CoM, Kp_joints, Reg, Gain)
    %BALANCING CONTROLLER

    %% DEFINITION OF CONTROL AND DYNAMIC VARIABLES
    pos_leftFoot   = w_H_l_sole(1:3,4);
    w_R_l_sole     = w_H_l_sole(1:3,1:3);
    pos_rightFoot  = w_H_r_sole(1:3,4);
    w_R_r_sole     = w_H_r_sole(1:3,1:3);

    Kd_joints      = Gain.Kd_joints;
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
    xDot_CoM       = J_CoM(1:3,:)*nu;
    
    % Joint velocity
    sDot           = nu(7:end);
    
    % Joint position error
    s_tilde        =  s-s_des;
    
    % Desired acceleration for the center of mass
    xDDot_CoM_star = desired_x_xDot_xDDot_CoM(:,3) -Kp_CoM.*(x_CoM - desired_x_xDot_xDDot_CoM(:,1)) -Kd_CoM.*(xDot_CoM - desired_x_xDot_xDDot_CoM(:,2));
   
    % Application point of the contact force on the right foot w.r.t. CoM
    Pr              = pos_rightFoot -x_CoM; 
    
    % Application point of the contact force on the left foot w.r.t. CoM
    Pl              = pos_leftFoot  -x_CoM; 

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
    
    pinvA           = pinv(A, Reg.pinvTol)*feet_in_contact(1)*feet_in_contact(2)  ...
                      + [inv(AL);zeros(6)]*feet_in_contact(1)*(1-feet_in_contact(2)) ... 
                      + [zeros(6);inv(AR)]*feet_in_contact(2)*(1-feet_in_contact(1)); 
                
    % Null space of the matrix A            
    Null_A          = (eye(12,12)-pinvA*A)*feet_in_contact(1)*feet_in_contact(2);

    % Time varying contact jacobian
    Jc              = [J_l_sole*feet_in_contact(1);      
                       J_r_sole*feet_in_contact(2)];
                   
    % Time varying dot(J)*nu
    JcDot_nu        = [JDot_l_sole_nu*feet_in_contact(1) ;      
                       JDot_r_sole_nu*feet_in_contact(2)];

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
    Kp_joints       = diag(Kp_joints)*pinv(NLMbar,Reg.pinvTol) + Reg.Kp_joints_tol*eye(ROBOT_DOF);
    Kd_joints       = diag(Kd_joints)*pinv(NLMbar,Reg.pinvTol) + Reg.Kd_joints_tol*eye(ROBOT_DOF); 
  
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
    tau_model  = Pinv_JcMinvSt*(JcMinv*h - JcDot_nu) + nullJcMinvSt*(h(7:end) - Mbj'/Mb*h(1:6) ...
                -Kp_joints*NLMbar*s_tilde -Kd_joints*NLMbar*sDot);
    
    Sigma      = -(Pinv_JcMinvSt*JcMinvJct + nullJcMinvSt*JBar);
    
    % Desired rate-of-change of the robot momentum
    HDotDes    = [m*xDDot_CoM_star ;
                 -Gain.Kp_AngularMomentum*L(4:end)-Gain.Ki_AngularMomentum*intL(4:6)];

    % Contact wrenches realizing the desired rate-of-change of the robot
    % momentum HDotDes when standing on two feet. Note that f_HDot is
    % different from zero only when both foot are in contact, i.e. 
    % constraints(1) = constraints(2) = 1. This because when the robot
    % stands on one foot, the f_HDot is evaluated directly from the
    % optimizer (see next section).
    f_HDot_des    = pinvA*(HDotDes - gravityWrench)*feet_in_contact(1)*feet_in_contact(2);
    SigmaNA       = Sigma*Null_A;
   
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
    ConstraintsMatrix_QP2Feet  = ConstraintsMatrix2Feet*Null_A;
    bVectorConstraints_Qp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot_des;
    
    % Evaluation of Hessian matrix and gradient vector for solving the
    % optimization problem 1).
    HessianMatrix_QP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*Reg.HessianQP;
    gradient_QP2Feet           = SigmaNA'*(tau_model + Sigma*f_HDot_des);

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
    ConstraintsMatrix_QP1Foot  = feet_in_contact(1) * (1 - feet_in_contact(2)) * constraintMatrixLeftFoot + ...
                                 feet_in_contact(2) * (1 - feet_in_contact(1)) * constraintMatrixRightFoot;
    bVectorConstraints_Qp1Foot = bVectorConstraints;

    A1Foot                     =  AL*feet_in_contact(1)*(1-feet_in_contact(2)) + AR*feet_in_contact(2)*(1-feet_in_contact(1));
    HessianMatrix_QP1Foot      =  A1Foot'*A1Foot + eye(size(A1Foot,2))*Reg.HessianQP;
    gradient_QP1Foot           = -A1Foot'*(HDotDes - gravityWrench);

    %% DEBUG DIAGNOSTICS
   
    % Error on the center of mass
    CoM_error                 =  x_CoM - desired_x_xDot_xDDot_CoM(:,1);
end
