function references_CoM = referenceGeneratorCoM(posCoM_0, t, Config)

    % REFERENCEGENERATORCOM generates sinusoidal references for the robot
    %                       CoM position, velocity and acceleration.

    %% --- Initialization ---

    ampl = Config.referencesCoM.amplitude;
    freq = Config.referencesCoM.frequency;
    dir  = Config.referencesCoM.direction;

    % CoM references
    posCoM_des     =  posCoM_0 + ampl * sin(2*pi*freq*t) * dir;
    velCoM_des     =  ampl * 2 * pi * freq * cos(2*pi*freq*t) * dir;
    accCoM_des     = -ampl * 4 * pi^2 * freq^2 * sin(2*pi*freq*t) * dir;

    references_CoM = [posCoM_des; velCoM_des; accCoM_des];
end
