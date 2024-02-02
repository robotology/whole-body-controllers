function [M_c,C_c_nu_c,g_c,Jc_c,dJc_nu_c,nu_c] = fromFloatingToCentroidalDynamics(M, h, g, Jc, dJc_nu, nu, T, dT)
    %FROMFLOATINGTOCENTROIDALDYNAMICS  converts dynamic equation parameters to the
    %                       corresponding values in centroidal frame of reference.
    %
    % [M_c,C_c_nu_c,g_c,Jc_c,dJc_nu_c,nu_c] = FROMFLOATINGTOCENTROIDALDYNAMICS
    % (M, h, g, Jc, dJc_nu, nu, T, dT) takes as an input the robot dynamics.
    %  The output is the robot dynamics in centroidal frame of reference.
    %

    % ------------Initialization----------------
    ndof   = size(g,1)-6;
    invT   = eye(ndof+6)/T;
    invTt  = eye(ndof+6)/(T');

    %% Control terms conversion
    % mass matrix
    M_c            = invTt*M*invT;
    M_c(1:6,7:end) = zeros(6,ndof);
    M_c(7:end,1:6) = zeros(ndof,6);
    M_c(1:3,1:3)   = M(1,1)*eye(3);
    M_c(1:3,4:6)   = zeros(3);
    M_c(4:6,1:3)   = zeros(3);
    Mb             = M(1:6,1:6);
    Mbj            = M(1:6,7:end);

    nu_c           = T*nu;
    gravAcc        = norm(invTt*g)/M(1,1);
    e3             = zeros(ndof+6,1);
    e3(3)          = 1;
    g_c            = M(1,1)*gravAcc*e3;

    %coriolis terms
    C_nu           = h - g;
    C_nu_j         = C_nu(7:end);
    C_nu_b         = C_nu(1:6);
    C_c_nu_c_dT    = invTt*C_nu - M_c*dT*nu;
    C_c_nu_c       = [ zeros(3,1);
        C_c_nu_c_dT(4:6);
        C_nu_j-(Mbj')*(Mb\C_nu_b)];

    %new dT*nu computation for Jacobian
    dT_nu          = M_c\(C_nu-C_c_nu_c); % M_c\(inv(T)*C_nu-C_c_nu_c);
    Jc_c           = Jc*invT;
    dJc_nu_c       = dJc_nu - Jc*invT*dT_nu;

end
