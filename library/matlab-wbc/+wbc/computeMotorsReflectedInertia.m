function reflectedInertia = computeMotorsReflectedInertia(Gamma,T,I_m)

    % COMPUTEMOTORSREFLECTEDINERTIA compute the motors reflected inertia for 
    %                               a mechanical system composed of motors,
    %                               links and RIGID transmissions.
    %
    % FORMAT: reflectedInertia = computeMotorsReflectedInertia(Gamma,T,I_m)
    %
    % INPUT:  - Gamma: [n * n] diagonal matrix that accounts for the transmission 
    %                          ratio of the joints in the mechanism;
    %         - T: [n * n] matrix that accounts for the coupling between
    %                      different joints;
    %         - I_m: [n * n] diagonal matrix that contains the motors
    %                        inertia (not reflected).
    %
    % OUTPUT: - reflectedInertia: [n * n] matrix of motors reflected inertia
    %
    % Authors: Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

     % compute motors reflected inertia    
     invTGamma          = eye(size(Gamma))/(T*Gamma);
     invTGamma_t        = eye(size(Gamma))/(transpose(T*Gamma));
     reflectedInertia   = invTGamma_t*I_m*invTGamma;     
end