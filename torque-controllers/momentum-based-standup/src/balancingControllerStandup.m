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
function [tau_model, Sigma, NA, f_HDot, ...
          HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot, bVectorConstraintsQp1Foot, ...
          HessianMatrixQP2FeetOrLegs, gradientQP2FeetOrLegs, ConstraintsMatrixQP2FeetOrLegs, bVectorConstraintsQp2FeetOrLegs, ...
          CoM_error, correctionFromSupportForce, H_error, V] =  ...
              balancingControllerStandup(feet_in_contact, ROBOT_DOF_FOR_SIMULINK, ConstraintsMatrix, bVectorConstraints, ...
                                         s, s_des, nu, M, h, L, intL, w_H_l_contact, w_H_r_contact, JL, JR, dJL_nu, dJR_nu,...
                                         x_CoM, J_CoM, desired_x_xDot_xDDot_CoM, Kp_CoM, Kd_CoM, Kp_joints, Reg, Gain, w_H_lArm, w_H_rArm, LArmWrench, RArmWrench, STANDUP_WITH_HUMAN, state)
    % BALANCING CONTROLLER

    %% DEFINITION OF CONTROL AND DYNAMIC VARIABLES
    pos_leftContact    = w_H_l_contact(1:3,4);
    w_R_l_contact      = w_H_l_contact(1:3,1:3);
    pos_rightContact   = w_H_r_contact(1:3,4);
    w_R_r_contact      = w_H_r_contact(1:3,1:3);
    
    % arms position
    pos_leftArm     = w_H_lArm(1:3,4);
    pos_rightArm    = w_H_rArm(1:3,4);
    
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
                    
    % Application point of the contact force on the right contact link w.r.t. CoM
    Pr              = pos_rightContact -x_CoM; 
    
    % Application point of the contact force on the left contact link w.r.t. CoM
    Pl              = pos_leftContact  -xCoM; 
    
    % Arms location w.r.t. CoM
    PlArm           = pos_leftArm   -x_CoM; 
    PrArm           = pos_rightArm  -x_CoM;

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
    
    % matrix multipliyng the forces at arms
    A_lArm          = [ eye(3),     zeros(3);
                        skew(PlArm),  eye(3)];    
    A_rArm          = [ eye(3),     zeros(3);
                        skew(PrArm),  eye(3)];  
                    
    A_arms          = [A_lArm, A_rArm]; 
    
    pinvA           = pinv(A, Reg.pinvTol)*constraints(1)*constraints(2)  ...
                      + [inv(AL);zeros(6)]*constraints(1)*(1-constraints(2)) ... 
                      + [zeros(6);inv(AR)]*constraints(2)*(1-constraints(1)); 

    % Null space of the matrix A            
    NA              = (eye(12,12)-pinvA*A)*constraints(1)*constraints(2);

    % Total wrench applied at arms. External force is decomposed into two components
    % along the direction of momentum error. Then, the parallel component is considered
    fArms           = [LArmWrench;
                       RArmWrench];
                   
    % support force               
    fsupport        = A_arms * fArms;
    
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
                                    
    % desired robot momentum
    H_desired  = [m.*desired_x_dx_ddx_CoM(:,2);
                  zeros(3,1)];
              
    % momentum error
    H_error    = H -H_desired;
    
    % projector of contact forces into the direction parallel to momentum
    % error
    alpha         = (transpose(H_error)*fsupport)/(norm(H_error)+Reg.norm_tolerance);
    H_errParallel = H_error/(norm(H_error)+Reg.norm_tolerance);
    
    if STANDUP_WITH_HUMAN && alpha <= 0 && state < 4
        
        correctionFromSupportForce = alpha*H_errParallel;
    else
        correctionFromSupportForce = zeros(6,1);
    end                
                
    %% QP PARAMETERS FOR TWO FEET STANDING
    % In the case the robot stands on two feet/legs, the control objective is 
    % the minimization of the joint torques through the redundancy of the 
    % contact forces. By direct calculations one shows that the joint
    % torques take the following form:
    %
    % 0) tau = tauModel + Sigma*f_HDot + SigmaNA*f0
    %
    % where f0 is the redundancy of the contact wrenches. Then, the problem
    % is defined as follows:
    %
    % 1) f0  = argmin |tau(f0)|^2
    %          s.t.
    %          ConstraintsMatrixQP2Contacts*f0 < bVectorConstraintsQp2Contacts
    %
    % Update constraint matrices. The constraint matrix for the inequality
    % constraints in the problem 1) is built up starting from the constraint
    % matrix associated with each single contact. More precisely, the contact
    % wrench associated with the left contact (resp. right contact) is subject to
    % the following constraint:
    %
    % constraintMatrixLeftContact*l_sole_f_L < bVectorConstraints
    %
    % In this case, however, f_L is expressed w.r.t. the frame l_contact,
    % which is solidal to the left contact. Since the controller uses contact
    % wrenches expressed w.r.t. a frame whose orientation is that of the
    % inertial frame, we have to update the constraint matrix according to
    % the transformation w_R_l_contact, i.e.
    %
    % constraintMatrixLeftContact = ConstraintsMatrix*w_R_l_contact
    %
    % The same hold for the right foot
    constraintMatrixLeftContact   = ConstraintsMatrix * blkdiag(w_R_l_contact',w_R_l_contact');
    constraintMatrixRightContact  = ConstraintsMatrix * blkdiag(w_R_r_contact',w_R_r_contact');
    ConstraintsMatrix2FeetOrLegs  = blkdiag(constraintMatrixLeftContact,constraintMatrixRightContact);
    bVectorConstraints2FeetOrLegs = [bVectorConstraints;bVectorConstraints];
    
     % Terms used in Eq. 0)
    tauModel  = Pinv_JcMinvSt*(JcMinv*h - Jc_nuDot) + nullJcMinvSt*(h(7:end) - Mbj'/Mb*h(1:6) ...
               -impedances*NLMbar*qjTilde -dampings*NLMbar*qjDot);
    
    Sigma     = -(Pinv_JcMinvSt*JcMinvJct + nullJcMinvSt*JBar);
    
    % Desired rate-of-change of the robot momentum
    HDotDes   = [m*xDDcomStar ;
                -Gain.KD_AngularMomentum*H(4:end)-Gain.KP_AngularMomentum*intHw] +correctionFromSupportForce;

    % computing Lyapunov function
    int_H_tilde_times_gain = [m * gainsPCOM .* (xCoM - desired_x_dx_ddx_CoM(:,1));
                              Gain.KP_AngularMomentum .* intHw];
                          
    V = 0.5 * transpose(H_error) * H_error;
    V = V + 0.5 * transpose(int_H_tilde_times_gain) * int_H_tilde_times_gain;
    
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
    ConstraintsMatrixQP2FeetOrLegs  = ConstraintsMatrix2FeetOrLegs*NA;
    bVectorConstraintsQp2FeetOrLegs = bVectorConstraints2FeetOrLegs-ConstraintsMatrix2FeetOrLegs*f_HDot;
    
    % Evaluation of Hessian matrix and gradient vector for solving the
    % optimization problem 1).
    HessianMatrixQP2FeetOrLegs      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*Reg.HessianQP;
    gradientQP2FeetOrLegs           = SigmaNA'*(tauModel + Sigma*f_HDot);

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
    ConstraintsMatrixQP1Foot  = constraints(1) * (1 - constraints(2)) * ConstraintsMatrix + ...
                                constraints(2) * (1 - constraints(1)) * ConstraintsMatrix;
    bVectorConstraintsQp1Foot = bVectorConstraints;

    A1Foot                    =  AL*constraints(1)*(1-constraints(2)) + AR*constraints(2)*(1-constraints(1));
    HessianMatrixQP1Foot      =  A1Foot'*A1Foot + eye(size(A1Foot,2))*Reg.HessianQP;
    gradientQP1Foot           = -A1Foot'*(HDotDes - gravityWrench);

    %% DEBUG DIAGNOSTICS
    
    % Unconstrained solution for the problem 1)
    f0                        = -pinvDamped(SigmaNA, Reg.pinvDamp*1e-5)*(tauModel + Sigma*f_HDot);
    
    % Unconstrained contact wrenches
    f_noQP                    =  pinvA*(HDotDes - gravityWrench) + NA*f0*constraints(1)*constraints(2); 
    
    % Error on the center of mass
    errorCoM                  =  xCoM - desired_x_dx_ddx_CoM(:,1);
end
