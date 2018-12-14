function onOneFoot = contactsTransitionQP(LR_FootInContact)

    % CONTACTSTRANSITIONQP detects a contact activation (from left foot to
    %                      right foot balancing) and provides this information
    %                      to the WBtoolbox QP block's utilities.
    %                                        
    % FORMAT: onOneFoot = contactsTransitionQP(LR_FootInContact)
    %
    % INPUT:   - LR_FootInContact = a vector describing the feet contact status;
    %
    % OUTPUT:  - onOneFoot = true if the robot is balancing on one foot, false
    %                        otherwise.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

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
