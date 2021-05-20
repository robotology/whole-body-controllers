classdef EventWithJointLimits < Simulink.IntEnumType
    %EVENTWITHJOINTLIMITS Class for selecting a None, Warning or EmergencyStop signal
    %
    %   Class for selecting a None, Warning or EmergencyStop signal triggered by the joints hitting
    %   the limits.
    
    enumeration
        None(0)
        Warning(1)
        EmergencyStop(2)
    end
end
