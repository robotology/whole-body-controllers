function onOneFoot = contactsTransitionQP(LR_FootInContact)

CONTACT_THRESHOLD = 0.1;

if sum(LR_FootInContact) > (2 - CONTACT_THRESHOLD)
    % 2 Contacts
    onOneFoot = false;
    return;
elseif sum(LR_FootInContact) > (1 - CONTACT_THRESHOLD)
    % 1 Contact
    onOneFoot = true;
    return;
else
    % TODO
    onOneFoot = false;
    return;
end
