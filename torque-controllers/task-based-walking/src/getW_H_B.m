% GETW_H_B selects the base to world transform according to the foot that
%          is considered fixed on ground
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: w_H_b = getW_H_B(w_H_LFoot, w_H_RFoot, b_H_LFoot, b_H_RFoot, leftIsFixed)
%
% INPUT:  - b_H_LFoot = [4 * 4] base to LFoot transform
%         - b_H_RFoot = [4 * 4] base to RFoot transform
%         - w_H_LFoot = [4 * 4] world to LFoot transform
%         - w_H_RFoot = [4 * 4] world to RFoot transform
%         - lefitIsFixed = boolean for checking if the left foot is fixed on ground
%
% OUTPUT: - w_H_b = [4 * 4] world to base transform
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function w_H_b = getW_H_B(w_H_LFoot, w_H_RFoot, b_H_LFoot, b_H_RFoot, leftIsFixed)

    if round(leftIsFixed) == 1
        w_H_b = w_H_LFoot / b_H_LFoot;
    else
        w_H_b = w_H_RFoot / b_H_RFoot;
    end
end