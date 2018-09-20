%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables
clc

% add the path to the static gui, and open the model
addpath('../../library/matlab-gui');

% open the model
open_system('torqueBalancingYoga.mdl','loadonly');

% waiting for the user to close the model
closeModel = input('ENTER ANY NUMBER/STRING TO CLOSE THE MODEL ');

% Save the model and close
if exist('closeModel','var')
    
    save_system('torqueBalancingYoga.mdl');
    close_system('torqueBalancingYoga.mdl');
    rmpath('../../library/matlab-gui');
end

