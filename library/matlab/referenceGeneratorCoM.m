% REFERENCEGENERATORCOM computes a CoM reference trajectory. Default trajectory is
%                       a sine function.
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: references_CoM = referenceGeneratorCoM(pos_CoM_0, t, Config)
%
% INPUT:  - pos_CoM_0 = [3 * 1] initial CoM position
%         - t = simulation time
%         - Config = user defined configuration
%
%
% OUTPUT: - references_CoM = [9 * 1] desired CoM position, velocity and acceleration
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function references_CoM = referenceGeneratorCoM(pos_CoM_0, t, Config)
    
    pos_CoM_des = pos_CoM_0;
    vel_CoM_des = zeros(3,1);
    acc_CoM_des = zeros(3,1);

    if Config.amplitudeOfOscillation ~= 0
        
        if t > Config.noOscillationTime
        
            Amplitude = Config.amplitudeOfOscillation;
        else
            Amplitude = 0;
        end
        
        frequency   = Config.frequencyOfOscillation;
        pos_CoM_des = pos_CoM_0 +Amplitude*sin(2*pi*frequency*t)*Config.directionOfOscillation;
        vel_CoM_des =            Amplitude*2*pi*frequency*cos(2*pi*frequency*t)*Config.directionOfOscillation;
        acc_CoM_des =           -Amplitude*(2*pi*frequency)^2*sin(2*pi*frequency*t)*Config.directionOfOscillation;
    end
    
    references_CoM = [pos_CoM_des; vel_CoM_des; acc_CoM_des];
end
