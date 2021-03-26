function [T, dT] = centroidalTransformationT_TDot(xCoM,x_b,dxCoM,dx_b,M)
%CENTROIDALTRANSFORMATIONT_TDOT converts dynamic equation parameters to the
%                       corresponding values in centroidal frame of reference.
%
% [T, dT] = CENTROIDALTRANSFORMATIONT_TDOT(xCoM,x_b,dxCoM,dx_b,M)
% takes as an input the robot forward kinematics and the mass matrix M.
% The output are the transformation matrix T and its derivative, dT.
%% T calculation
r       = xCoM - x_b;

X       = [eye(3),wbc.skew(r)';
           zeros(3),eye(3)];

Mbj     = M(1:6,7:end);
Mb      = M(1:6,1:6);
Js      = X*(Mb\Mbj);

ndof    = size(Mbj,2);

T       = [X,Js;
           zeros(ndof,6),eye(ndof)];

%% Time derivative of T
dr      = dxCoM - dx_b;
mdr     = M(1,1)*dr;

dX      = [zeros(3),wbc.skew(dr)';
           zeros(3),zeros(3)];

dMb     = [zeros(3),wbc.skew(mdr)';
           wbc.skew(mdr),zeros(3)];

inv_dMb = -Mb\dMb/Mb;

dJs     = dX*(Mb\Mbj) + X*inv_dMb*Mbj;

dT      = [dX,dJs;
           zeros(ndof,6),zeros(ndof)];

end
