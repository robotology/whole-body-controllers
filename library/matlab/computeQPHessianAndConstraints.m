% COMPUTEQPHESSIANANDCONSTRAINTS computes Hessian matrix, gradient and
%                                constraints for integration based inverse 
%                                kinematics.
%
% IMPLEMENTATION: we assume to have a desired set of joint accelerations
%                 sDDot_star. We also assume to have a desired set of
%                 accelerations for a Cartesian task, namely acc_task_star.
%                 The objective is to find a set of state accelerations nuDot 
%                 which minimize the error (sDDot-sDDot_star), while subject 
%                 to the constraint that the task desired accelerations must 
%                 be achieved. The optimization procedure can be written as:
%
%                     nuDot = [nuDot_b; sDDot];
%
%                     nuDot = min(1/2*|(sDDot-sDDot_star) + nuDot_b|^2)
%
%                         s.t. J*nuDot + JDot_nu = acc_task_star                     
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [Hessian, gradient, ConstraintMatrix, biasVectorConstraint] = ...
%             computeQPHessianAndConstraints(s, nu, s_sDot_sDDot_ref, ...
%                                            impedances, dampings, J, JDot_nu, ...
%                                            acc_task_star) 
%
% INPUT:  - s = [ROBOT_DOF * 1]joint positions
%         - nu = [ROBOT_DOF + 6 * 1] state velocities
%         - s_sDot_sDDot_ref = [ROBOT_DOF * 3] joint unconstrained references
%         - impedances = [ROBOT_DOF * ROBOT_DOF] joint position gains
%         - dampings = [ROBOT_DOF * ROBOT_DOF] joint velocity gains
%         - J = [n_c * ROBOT_DOF + 6] task Jacobian
%         - JDot_nu = [n_c * 1] task Jacobian derivative times state velocities
%         - acc_task_star = [n_c * 1] task desired accelerations 
%
% OUTPUT: - Hessian = [ROBOT_DOF + 6 * ROBOT_DOF + 6] Hessian matrix 
%         - gradient = [ROBOT_DOF + 6 * 1] gradient for QP optimization
%         - ConstraintMatrix = [ n_c * ROBOT_DOF + 6] constraint matrix
%         - biasVectorConstraint = [ n_c * 1] bias vector constraints
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function [Hessian, gradient, ConstraintMatrix, biasVectorConstraint] = ...
             computeQPHessianAndConstraints(s, nu, s_sDot_sDDot_ref, impedances, dampings, J, JDot_nu, acc_task_star)
      
     % compute joint velocities    
     sDot    = nu(7:end);
         
     % compute Hessian matrix
     Hessian = eye(size(s,1) +6);
     
     % compute gradient
     sDDot_star =  s_sDot_sDDot_ref(:,3) ...
                  -diag(impedances)*(s -s_sDot_sDDot_ref(:,1)) ...
                  -diag(dampings)*(sDot -s_sDot_sDDot_ref(:,2));                      

     % to ensure the convergence of the base accelerations to zero, a feedback
     % term on base valocities is considered. the parameter k is a positive gain.
     k          = 1;
     gradient   = [k*nu(1:6); -sDDot_star];
     
     % compute equality constraint matrix
     ConstraintMatrix = J;
    
     %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %% TEMPORARY FIX FOR SOLVING A BUG IN QPOAES WITH EQUALITY CONSTRAINTS
     %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     correction = 1e-9.*size(3, size(s,1) +6);
     ConstraintMatrix(16:18,:) = ConstraintMatrix(16:18,:) -correction;
     %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     % compute bias vector constraints
     biasVectorConstraint = acc_task_star -JDot_nu;
end