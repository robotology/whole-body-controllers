clear variables
clc

addpath(genpath('../../library'));

open_system('torqueBalancingYoga.mdl','loadonly');

a = input('Waiting for the user to close the model: ');


if a == 1
    
    close_system('torqueBalancingYoga.mdl');
end


stopTorqueBalancingYoga