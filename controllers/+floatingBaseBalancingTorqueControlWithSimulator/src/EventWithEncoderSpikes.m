classdef EventWithEncoderSpikes < Simulink.IntEnumType
    %EVENTWITHENCODERSPIKES Class for selecting a None, Warning or EmergencyStop signal
    %   
    %   Class for selecting a None, Warning or EmergencyStop signal triggered by an encoder spike
    %   event.
    
    enumeration
        None(0)
        Warning(1)
        EmergencyStop(2)
    end
end
