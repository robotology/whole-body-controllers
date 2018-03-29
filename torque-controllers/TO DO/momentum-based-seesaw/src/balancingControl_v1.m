function [fNoQP,fHDot,NA,tauModel,SIGMA_fH,SIGMA_NA,HessianMatrixQP2Feet,gradientQP2Feet,...
          ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet] = ...
              ...
              balancingControl_v1(M_r,M_s,h_r,h_s,J_r,J_f,J_s,J_rDot_nu,J_fDot_nu_s,J_sDot_nu_s,w_H_lSole,w_H_rSole,w_p_CoM_r,...
                                  J_CoM_r,nu,gain,desired_x_dx_ddx_CoM_r,H_r,w_R_s,H_s,...
                                  reg,ConstraintsMatrix,bVectorConstraints,ROBOT_DOF,qj,qjDes,s_p_CoM_s,desired_x_CoM_s,s_omega,seesaw,intH_r)

% CONTROL # 1: linear momentum of the robot and angular momentum of the whole system

% Constants and common parameters
e3 = [0;0;1];
g = -9.81;
qjDot = nu(7:end);
delta = seesaw.delta;
rho = seesaw.rho;
w_omega = w_R_s*s_omega;
w_r = (delta * w_R_s * e3) -rho * e3;
w_v_CoM_s = skew(w_r) * w_omega;
w_R_s_bar = blkdiag(w_R_s,w_R_s);

% Dimension of the joint configuration space
nDof = size(ROBOT_DOF,1);

% Total Jacobian derivative times total velocity
JDot_nu = J_rDot_nu -J_fDot_nu_s;

% Total control torques selector
S = [zeros(6,nDof);
     eye(nDof,nDof)];

%% Linear and angular momentum of the robot

w_p_lSole = w_H_lSole(1:3,4);
w_p_rSole = w_H_rSole(1:3,4);

w_gl = w_p_lSole - w_p_CoM_r;
w_gr = w_p_rSole - w_p_CoM_r;

% gravity wrench in world frame   
gravityWrench_r = [(M_r(1,1)*g*e3);zeros(3,1)];

% Matrix which projects forces into the robot momentum dynamics
AR = [eye(3)      zeros(3)  eye(3)      zeros(3);
      skew(w_gl)  eye(3)    skew(w_gr)  eye(3) ];

% linear velocity of the robot CoM 
w_nu_CoM_r = J_CoM_r * nu;
w_v_CoM_r = w_nu_CoM_r(1:3);

% saturate the robot CoM position error
saturated_xCoM = saturate(gain.PCOM * (desired_x_dx_ddx_CoM_r(:,1)-w_p_CoM_r),-gain.P_SATURATION,gain.P_SATURATION);
  
% desired robot CoM acceleration
ddxCoM_star_r = desired_x_dx_ddx_CoM_r(:,3) + saturated_xCoM + gain.DCOM * (desired_x_dx_ddx_CoM_r(:,2)-w_v_CoM_r);

% robot desired linear and angular momentum
H_rDot_star = [(M_r(1,1) * ddxCoM_star_r); 
               (-gain.DAngularMomentum * H_r(4:end) -gain.PAngularMomentum * intH_r(4:end))];
              

%% Linear and angular momentum of the seesaw 

% Matrix which projects forces into the seesaw momentum dynamics
invM_s =  eye(6)/M_s;
AS = -transpose(J_f) +transpose(J_s)/(J_s*invM_s*transpose(J_s))*J_s*invM_s*transpose(J_f);

% bias forces in seesaw frame
biasForces_s = transpose(J_s)/(J_s*invM_s*transpose(J_s))*(J_s*invM_s*h_s -J_sDot_nu_s);

% seesaw CoM position
w_p_CoM_s = w_R_s * s_p_CoM_s;

%% Linear and angular momentum of the whole system

% total mass
mT = M_s(1,1)+M_r(1,1);

% total CoM position
w_p_CoM_t = (M_r(1,1)*w_p_CoM_r + M_s(1,1)*w_p_CoM_s)/mT;

% desired total CoM position
w_p_CoM_tDes = (M_r(1,1)*desired_x_dx_ddx_CoM_r(:,1) + M_s(1,1)*desired_x_CoM_s)/mT;

% total velocity
w_v_CoM_t = (M_r(1,1)*w_v_CoM_r + M_s(1,1)*w_v_CoM_s)/mT; 

% projectors of momenta in the total momentum equation
b_X_s_star = [eye(3) zeros(3);
              skew(w_p_CoM_s-w_p_CoM_t) eye(3)];
          
b_X_r_star = [eye(3) zeros(3);
              skew(w_p_CoM_r-w_p_CoM_t) eye(3)];
       
% total momentum
H_t = b_X_r_star*H_r + b_X_s_star*w_R_s_bar*H_s;

% bias forces
biasForces_t = [mT*g*e3; zeros(3,1)] + b_X_s_star*w_R_s_bar*biasForces_s;

% saturate the  CoM position error
saturated_xCoM = saturate(gain.PCOM * (w_p_CoM_tDes-w_p_CoM_t),-gain.P_SATURATION,gain.P_SATURATION);
  
% desired CoM acceleration
ddxCoM_star_t = saturated_xCoM -gain.DCOM * w_v_CoM_t;

% desired linear and angular momentum
H_tDot_star = [(mT * ddxCoM_star_t); 
               (-gain.DAngularMomentum * H_t(4:end))];
 
% forces  multiplier              
AT =  b_X_s_star*w_R_s_bar*transpose(J_s)/(J_s*invM_s*transpose(J_s))*J_s*invM_s*transpose(J_f);  
 
%% Desired forces 

% global forces multiplier and its nullspace
A = [AR(1:3,:);AT(4:end,:)];

pinvA = pinv(A,reg.pinvTol);
NA = eye(size(pinvA*A)) - pinvA*A;

HDot_star = [(H_rDot_star(1:3)-gravityWrench_r(1:3))
             (H_tDot_star(4:end)-biasForces_t(4:end))];
          
% forces and moments satisfying the momentum dynamics
fHDot = pinvA * HDot_star;

%% Control torques computation

% define the required parameters
JinvM = J_r/M_r;
JinvMS = JinvM * S;
JinvMJt = JinvM * transpose(J_r);
pinv_JinvMS = pinvDamped(JinvMS,reg.pinvDamp); 
NullJinvMS = eye(nDof) - pinv_JinvMS*JinvMS;

% multiplier of f in tau_0
JBar = transpose(J_r(:,7:end))-M_r(7:end,1:6)/M_r(1:6,1:6)*transpose(J_r(:,1:6));
   
tauModel = pinv_JinvMS*(JinvM*h_r - JDot_nu -J_f*invM_s*(h_s-biasForces_s)) + NullJinvMS*(h_r(7:end) -M_r(7:end,1:6)/M_r(1:6,1:6)*h_r(1:6) ...
          -gain.impedances*(qj-qjDes) -gain.dampings*qjDot);
  
SIGMA = -(pinv_JinvMS*(JinvMJt -J_f*invM_s*AS) + NullJinvMS*JBar);
 
%% Optimization using quadratic programming (QP) solver

% parameters from control and dynamics
SIGMA_NA = SIGMA * NA; 
SIGMA_fH = SIGMA * fHDot;
w_R_lSole = w_H_lSole(1:3,1:3);
w_R_rSole = w_H_rSole(1:3,1:3);

% QP parameters
constraintMatrixLeftFoot = ConstraintsMatrix * blkdiag(transpose(w_R_lSole),transpose(w_R_lSole));
constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(transpose(w_R_rSole),transpose(w_R_rSole));
ConstraintsMatrix2Feet = blkdiag(constraintMatrixLeftFoot,constraintMatrixRightFoot);
bVectorConstraints2Feet = [bVectorConstraints;bVectorConstraints];

ConstraintsMatrixQP2Feet = ConstraintsMatrix2Feet*NA;
bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*fHDot;
HessianMatrixQP2Feet = transpose(SIGMA_NA)*SIGMA_NA + eye(size(SIGMA_NA,2))*reg.HessianQP;
gradientQP2Feet = transpose(SIGMA_NA)*(tauModel + SIGMA*fHDot);

fNoQP = -pinvDamped(SIGMA_NA,reg.pinvDamp*1e-5)*(tauModel + SIGMA*fHDot);

end