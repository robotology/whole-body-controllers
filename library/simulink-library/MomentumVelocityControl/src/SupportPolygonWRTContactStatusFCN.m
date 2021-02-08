function CoM_limits = SupportPolygonWRTContactStatusFCN(contactStatus,Config)
CoM_limits = Config.CoM_limit_double;

if(and(contactStatus(1),contactStatus(2))) % Double Support
    CoM_limits = Config.CoM_limit_double;
elseif(and(~contactStatus(1),~contactStatus(2))) % Flying phase
    CoM_limits = Config.CoM_limit_double;
elseif(contactStatus(1)) % Left Foot contact
    CoM_limits =Config.CoM_limit_left;
elseif(contactStatus(2)) % Right Foot contact 
    CoM_limits = Config.CoM_limit_right;
end
end