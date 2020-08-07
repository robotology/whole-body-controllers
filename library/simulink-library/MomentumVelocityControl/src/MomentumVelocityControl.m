function  [H, g, A, ibA, ubA, LB, UB] = MomentumVelocityControl(J_L, J_R, J_CMM, x_com, x_com_des, L_d, int_L_angular, ...
                                                                ConstraintsMatrix, bVectorConstraints,s_k,s_des_k, s_dot_des_k_1, ...
                                                                M,h,nu_k,w_H_r_sole,w_H_l_sole,deltaT, Gains,Weigth, Sat)

% Joints degrees of freedom
NDOF        = length(s_des_k);

% Contact Jacobian Computation 
Jc          = [J_L; ...
               J_R];

% Selector of actuated DoFs
B           = [zeros(6,NDOF); ...
               eye(NDOF,NDOF)];

%Computing the total friction cones constraints
w_R_r_sole                 = w_H_r_sole(1:3,1:3);
w_R_l_sole                 = w_H_l_sole(1:3,1:3);
ConstraintMatrixLeftFoot   = ConstraintsMatrix * blkdiag(w_R_l_sole', w_R_l_sole');
ConstraintMatrixRightFoot  = ConstraintsMatrix * blkdiag(w_R_r_sole', w_R_r_sole');
ConstraintsMatrixBothFeet  = blkdiag(ConstraintMatrixLeftFoot,ConstraintMatrixRightFoot);
bVectorConstraintsBothFeet = [bVectorConstraints;bVectorConstraints];

% Compute References for postural and momentum tasks
[L_star, s_dot_k_1_star]   = wbc.ComputeReferences(x_com, x_com_des, L_d, int_L_angular, s_dot_des_k_1, s_des_k, s_k, Gains); 

%% Computing Tasks Matrices
  
% Postural 
[A_p, a_p] = wbc.ComputePosturalTask(s_dot_k_1_star, NDOF); 
% Compute Hessian and Gradient
[H_p, g_p] = wbc.ComputeHessianAndGradient(A_p,a_p, Weigth.PosturalTask); 

% Torque Minimization 
[A_tau, a_tau] = wbc.ComputeTorqueMinimizationTask(B, M, deltaT, Jc, h, nu_k); 
% Compute Hessian and Gradient
[H_tau, g_tau] = wbc.ComputeHessianAndGradient(A_tau,a_tau, Weigth.MinimizationTorques);

%% Compute Constraints 

% Equality 
[C_eq, b_eq] = wbc.ComputeEqualityConstraint(Jc, J_CMM, L_star); 
% Inequality 
[C_in, b_in] = wbc.ComputeInequalityConstraint(ConstraintsMatrixBothFeet, bVectorConstraintsBothFeet, NDOF); 

%% Compute QP Matrices 

% Compute Overall hessian and Gradient 
H   = H_p + H_tau; 
g   = g_p + g_tau; 
% Constraint Matrices 
A   = [C_eq;C_in];
ubA = [b_eq;b_in];
lower_bound = -inf*ones(size(b_in));% hack to have equality constraint 
ibA = [b_eq; lower_bound];

% Search Variable Limits
LB = [Sat.BaseLinearVelocity_min; Sat.BaseAngVelocity_min; Sat.JointsVelocity_min; Sat.wrenches_min];
UB = [Sat.BaseLinearVelocity_max; Sat.BaseAngVelocity_max; Sat.JointsVelocity_max; Sat.wrenches_max];

end
