function [A_tau, a_tau] = ComputeTorqueMinimizationTask(B, M, deltaT, J_F, h, nu_k); 

    A_tau = [(B\M)/ deltaT, -B\J_F'];
    a_tau = -B\h+ B\(M*nu_k/deltaT); 
end
