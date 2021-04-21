function wrench = fromParametrizationToForces(xi, Config)

    % delta feet size
    delta_x   =  (Config.feet_size(2,2) - Config.feet_size(2,1))/2;
    delta_x_0 = -(Config.feet_size(2,2) + Config.feet_size(2,1))/2;
    delta_y   =  (Config.feet_size(1,2) - Config.feet_size(1,1))/2;
    delta_y_0 =  (Config.feet_size(1,2) + Config.feet_size(1,1))/2;
    
    % linear forces
    Fz = exp(xi(3)) + Config.fZmin;
    Fx = sqrt(Config.forceFrictionCoefficient)*tanh(xi(1))*Fz/sqrt(1 + tanh(xi(2))^2);
    Fy = sqrt(Config.forceFrictionCoefficient)*tanh(xi(2))*Fz/sqrt(1 + tanh(xi(1))^2);
    
    % moments
    Mz = Config.torsionalFrictionCoefficient*tanh(xi(6))*Fz;
    Mx = (delta_x*tanh(xi(4)) + delta_x_0)*Fz;
    My = (delta_y*tanh(xi(5)) + delta_y_0)*Fz;
    
    % compute the final wrench
    wrench = [Fx; Fy; Fz; Mx; My; Mz];
end
