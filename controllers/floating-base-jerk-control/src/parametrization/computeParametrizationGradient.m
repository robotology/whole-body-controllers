function Beta = computeParametrizationGradient(xi, Config)

    % delta feet size
    delta_x   =  (Config.feet_size(2,2) - Config.feet_size(2,1))/2;
    delta_x_0 = -(Config.feet_size(2,2) + Config.feet_size(2,1))/2;
    delta_y   =  (Config.feet_size(1,2) - Config.feet_size(1,1))/2;
    delta_y_0 =  (Config.feet_size(1,2) + Config.feet_size(1,1))/2;
    
    % initialize Beta and Fz
    Beta      = zeros(6);
    Fz        = exp(xi(3)) + Config.fZmin;
    
    % add non-zero elements
    
    % df/dxi_1
    Beta(1,1) =  sqrt(Config.forceFrictionCoefficient)*(1 - tanh(xi(1))^2)*Fz/sqrt(1 + tanh(xi(2))^2);
    Beta(2,1) = -sqrt(Config.forceFrictionCoefficient)*tanh(xi(2))*Fz/(1 + tanh(xi(1))^2)^(3/2)*tanh(xi(1))*(1 - tanh(xi(1))^2);
    
    % df/dxi_2
    Beta(1,2) = -sqrt(Config.forceFrictionCoefficient)*tanh(xi(1))*Fz/(1 + tanh(xi(2))^2)^(3/2)*tanh(xi(2))*(1 - tanh(xi(2))^2);
    Beta(2,2) =  sqrt(Config.forceFrictionCoefficient)*(1 - tanh(xi(2))^2)*Fz/sqrt(1 + tanh(xi(1))^2);
    
    % df/dxi_3
    Beta(1,3) = sqrt(Config.forceFrictionCoefficient)*tanh(xi(1))*exp(xi(3))/sqrt(1 + tanh(xi(2))^2);
    Beta(2,3) = sqrt(Config.forceFrictionCoefficient)*tanh(xi(2))*exp(xi(3))/sqrt(1 + tanh(xi(1))^2);
    Beta(3,3) = exp(xi(3));
    Beta(4,3) = (delta_x*tanh(xi(4)) + delta_x_0)*exp(xi(3));
    Beta(5,3) = (delta_y*tanh(xi(5)) + delta_y_0)*exp(xi(3));
    Beta(6,3) = Config.torsionalFrictionCoefficient*tanh(xi(6))*exp(xi(3));
    
    % df/dxi_4
    Beta(4,4) = delta_x*(1 - tanh(xi(4))^2)*Fz;
    
    % df/dxi_5
    Beta(5,5) = delta_y*(1 - tanh(xi(5))^2)*Fz;
    
    % df/dxi_6
    Beta(6,6) = Config.torsionalFrictionCoefficient*(1 - tanh(xi(6))^2)*Fz;  
end
