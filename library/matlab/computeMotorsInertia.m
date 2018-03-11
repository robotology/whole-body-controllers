% COMPUTEMOTORSINERTIA compute the motors reflected inertia.
%
% FORMAT: reflectedInertia = computeMotorsInertia(Config)
%
% INPUT:  -Config = user defined configuration parameters
%
% OUTPUT: -reflectedInertia = [n x n] matrix of motors reflected inertia
%
% Authors: Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function reflectedInertia = computeMotorsInertia(Config)

     % compute motors reflected inertia
     Gamma               = Config.Gamma;
     T                   = Config.T;
     I_m                 = Config.I_m;     
     invTGamma           = eye(size(Gamma))/(T*Gamma);
     invTGamma_t         = eye(size(Gamma))/(transpose(T*Gamma));
     reflectedInertia    = invTGamma_t*I_m*invTGamma;     
end