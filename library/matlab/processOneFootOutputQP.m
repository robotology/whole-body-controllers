% PROCESSONEFOOTOUTPUTQP evaluates the output of the WBToolbox QP block, and 
%                        associates this output with the left foot contact
%                        forces or with the right foot contact forces (or
%                        both) according to the contacts status: if the
%                        contact is not active, a vector of zeros is
%                        assigned as contact forces and moments instead of
%                        the solution provided by the QP.
%
% FORMAT:  f0_oneFoot = processOneFootOutputQP(primalSolution,LR_FootInContact) 
%
% INPUT:  - primalSolution = the solution to the QP problem provided by the 
%                            WBToolbox QP block;
%         - LR_FootInContact = a vector describing the feet contact status;
%
% OUTPUT: - f0_oneFoot  = the final solution to the QP problem that will be used in
%                         the controller.
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function f0_oneFoot = processOneFootOutputQP(primalSolution,LR_FootInContact)

r_inContact = LR_FootInContact(1);
l_inContact = LR_FootInContact(2);
f0_oneFoot  = [primalSolution * r_inContact ; primalSolution * l_inContact]*abs(r_inContact - l_inContact);

end