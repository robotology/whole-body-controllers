function prevFootBoolean = checkPrevFootInContact(currentFootInContact)

    persistent prevFootInContact
    
    if isempty(prevFootInContact)
        
        prevFootInContact = currentFootInContact;
    end
    
    % boolean for selecting the integrator init conditions
    prevFootBoolean       = prevFootInContact;
    
    % the contact condition changed: update previous foot in contact
    if prevFootInContact ~= currentFootInContact
        
        prevFootInContact = currentFootInContact;
    end
end