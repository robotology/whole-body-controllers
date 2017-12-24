% BALANCINGCONTROLLER this function impements a control strategy for 
%                     balancing a constrained floating base system.
%
% IMPLEMENTATION: the robot is modeled as a floating base system:
%
%                     M(q)*nuDot + h(q,nu) = J_c(q)^T*f + S*tau  [1]
%
%                 subject to a set of holonomic constraints:
%
%                     J_c(q)*nuDot + J_cDot_nu = 0.  [2]
%
%                 For controlling the robot, a set of tasks in the
%                 Cartesian space is defined. In particular:
%                
%                 - position of the robot CoM;
%                 - orientation of a link specified by the user;
%                 - position and orientation of the swinging foot (ONE FOOT
%                   BALANCING ONLY).
%
%                 It is possible to relate the state accelerations and the
%                 cartesian accelerations through forward kinematics:
%
%                     J_t(q)*nuDot + J_tDot_nu = acc_t  [3]
%
%                 Then, we define as control input of eq. [1] both the 
%                 contact forces f and the joint torques tau. Eq. [1] becomes:
%
%                     M(q)*nuDot + h(q,nu) = B(q)*u  [4]
%
%                 where B(q) = [J(q)^T S]. We then define a postural task
%                 for controlling the joint space dynamics as follows:
%
%                     sDDot^* = sDDot_des -Kp*(s-s_des) -Kd*(sDot-sDot_des) [5].
%
%                 Finally, the whole control architecture can be rewritten
%                 as a constrained quadratic optimization. In fact, one may
%                 think of finding u that minimizes the error between
%                 sDDot and sDDot^* while satisfying [2]-[3], plus the 
%                 friction cones and unilateral constraints on the contact 
%                 forces. Furthermore, we aim at minimizing also the torques
%                 necessary for achieving the required task. In short:
%
%                     min_u |sDDot(u)-sDDot^* + K_t*tau|^2
% 
%                         s.t.  C*f < b
%                               J_c(q)*nuDot + J_cDot_nu = 0
%                               J_t(q)*nuDot + J_tDot_nu = acc_t
% 
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [Hessian,gradient,ConstraintMatrix_equality,biasVectorConstraint_equality, ...
%          ConstraintMatrix_inequality,biasVectorConstraint_inequality] = ...
%          balancingController(feetInContact, M, h, J, impedances, ...
%                              dampings, s, sDot, s_sDot_sDDot_des, acc_task_star, ...
%                              JDot_nu, ConstraintsMatrix_feet, biasVectorConstraint_feet, ...
%                              w_H_l_sole, w_H_r_sole, Sat)      
%
% INPUT: - feetInContact = [2 * 1] feet contact status (0 not active, 1 active)
%        - M = [n+6 * n+6] system mass matrix 
%        - h = [n+6 * 1] bias forces vector
%        - J = [n_t * n+6] tasks jacobian
%        - impedances = [n * n] position gains matrix 
%        - dampings = [n * n] velocity gains matrix
%        - s = [n * 1] joint positions
%        - sDot = [n * 1] joint velocities 
%        - s_sDot_sDDot_des = [n * 3] desired joint positions, velocities and accelerations
%        - acc_task_star = [n_t * 1] desired tasks accelerations
%        - JDot_nu = [n_t * n+6] tasks jacobian derivative times state velocities
%        - ConstraintsMatrix_feet = [19 * 6] friction cone + unilateral constr. on each foot
%        - biasVectorConstraint_feet = [19 * 1] bias vector for ineq. constraints
%        - w_H_l_sole = [4 * 4] world to left foot transformation matrix
%        - w_H_r_sole = [4 * 4] world to right foot transformation matrix
%        - Sat = structure containing user defined parameters
%
% OUTPUT: - Hessian = [n+6 * n+6] hessian matrix for QP solver
%         - gradient = [n+6 * 1] gradient for QP solver
%         - ConstraintMatrix_equality = [n_c * n+6] equality constraints matrix
%         - biasVectorConstraint_equality = [n_c * 1] equality constraints bias vector
%         - ConstraintMatrix_inequality = [n_c * 1] equality constraints matrix
%         - biasVectorConstraint_inequality = [n_c * n+6] equality constraints matrix
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function [Hessian,gradient,ConstraintMatrix_equality,biasVectorConstraint_equality, ...
          ConstraintMatrix_inequality,biasVectorConstraint_inequality] = ...
          balancingController(feetInContact, M, h, J, impedances, ...
                              dampings, s, sDot, s_sDot_sDDot_des, acc_task_star, ...
                              JDot_nu, ConstraintsMatrix_feet, biasVectorConstraint_feet, ...
                              w_H_l_sole, w_H_r_sole, Sat) 

    % Dimension of the joint space
    ROBOT_DOF = size(s,1);
    
    % Separating constraints from the Jacobian J. Structure of J:
    %
    %    J = [J_leftFoot; J_rightFoot; J_CoM; J_rot_task]
    %
    J_c = [J(1:6,:)*feetInContact(1);
           J(7:12,:)*feetInContact(2)];
    
    % Computing desired joint accelerations
    sDDot_star =  s_sDot_sDDot_des(:,3) ...
                 -diag(impedances)*(s-s_sDot_sDDot_des(:,1)) ....
                 -diag(dampings)*(sDot-s_sDot_sDDot_des(:,2));
    
    % Multiplier of u in the joint accelerations equation and bias terms
    S         = [zeros(6,ROBOT_DOF);
                 eye(ROBOT_DOF)];             
    B         = [transpose(J_c) S];
    St        = transpose(S);
    invM      = eye(ROBOT_DOF+6)/M;
    St_invM_B = St*invM*B;
    St_invM_h = St*invM*h;
    
    % Hessian matrix for minimizing also joint torques
    S_tau     = [zeros(ROBOT_DOF,12) eye(ROBOT_DOF)];
    
    % Hessian matrix and gradient for QP solver
    Hessian   =  transpose(St_invM_B)*St_invM_B + Sat.weight_tau*transpose(S_tau)*S_tau;
    gradient  = -transpose(St_invM_B)*(St_invM_h +sDDot_star);

    % Multiplier of u in the equality constraint equation and bias terms
    J_invM_B  = J*invM*B;
    J_invM_h  = J*invM*h;
    
    % Equality constraints
    ConstraintMatrix_equality     = J_invM_B;
    biasVectorConstraint_equality = acc_task_star -JDot_nu +J_invM_h;
    
    % Inequality constraints
    %
    % Update constraint matrices. The constraint matrix for the inequality
    % constraints is built up starting from the constraint matrix associated 
    % with each single foot. More precisely, the contact wrench associated 
    % with the left foot (resp. right foot) is subject to the following 
    % constraint:
    %
    %     constraintMatrixLeftFoot*l_sole_f_L < bVectorConstraints
    %
    % In this case, however, f_L is expressed w.r.t. the frame l_sole,
    % which is solidal to the left foot. Since the controller uses contact
    % wrenches expressed w.r.t. a frame whose orientation is that of the
    % inertial frame, we have to update the constraint matrix according to
    % the transformation w_R_l_sole, i.e.
    %
    %     constraintMatrixLeftFoot = ConstraintsMatrix*w_R_l_sole
    %
    % The same hold for the right foot
    %
    w_R_l_sole = w_H_l_sole(1:3,1:3);
    w_R_r_sole = w_H_r_sole(1:3,1:3);
  
    ConstraintMatrixLeftFoot  = ConstraintsMatrix_feet * blkdiag(transpose(w_R_l_sole),transpose(w_R_l_sole));
    ConstraintMatrixRightFoot = ConstraintsMatrix_feet * blkdiag(transpose(w_R_r_sole),transpose(w_R_r_sole));
  
    ConstraintMatrix_inequality     = blkdiag(feetInContact(1)*ConstraintMatrixLeftFoot, ...
                                              feetInContact(2)*ConstraintMatrixRightFoot);
    biasVectorConstraint_inequality = [feetInContact(1)*biasVectorConstraint_feet; ...
                                       feetInContact(2)*biasVectorConstraint_feet];
end
