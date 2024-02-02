function centroidalDyn = centroidalConversion(DYNAMICS,FORKINEMATICS,STATE)
    %CENTROIDALCONVERSION applies the centroidal coordinates tranformation to the
    %                     system, i.e. the base dynamics now coincides with the
    %                     CoM dynamics.
    %
    % centroidalDyn = CENTROIDALCONVERSION(DYNAMICS,FORKINEMATICS,STATE)
    % takes as an input the structures containing the robot dynamics,
    % forward kinematics and state.
    % The output is the structure CENTROIDALDYN which contains the robot
    % dynamics converted into the centroidal coordinates.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, May 2016

    % ------------Initialization----------------
    %% Dynamics
    M        = DYNAMICS.M;
    h        = DYNAMICS.h;
    g        = DYNAMICS.g;
    Jc       = DYNAMICS.Jc;
    dJc_nu   = DYNAMICS.dJc_nu;

    %% Forward kinematics
    xCoM     = FORKINEMATICS.xCoM;
    dxCoM    = FORKINEMATICS.dxCoM;

    %% Robot state
    nu       = STATE.nu;
    dx_b     = STATE.dx_b;
    x_b      = STATE.x_b;

    %% CENTROIDAL TRANSFORMATION
    [T,dT]                                    = centroidalTransformationT_TDot(xCoM,x_b,dxCoM,dx_b,M);
    [M_c, C_nu_c, g_c, Jc_c, dJc_nu_c, nu_c]  = fromFloatingToCentroidalDynamics(M, h, g, Jc, dJc_nu, nu, T, dT);

    centroidalDyn.M                           = M_c;
    centroidalDyn.g                           = g_c;
    centroidalDyn.C_nu                        = C_nu_c;
    centroidalDyn.dJc_nu                      = dJc_nu_c;
    centroidalDyn.nu                          = nu_c;
    centroidalDyn.Jc                          = Jc_c;

end
