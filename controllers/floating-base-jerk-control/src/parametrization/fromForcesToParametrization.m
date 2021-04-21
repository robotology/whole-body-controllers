function xi = fromForcesToParametrization(wrench, Config)

    % WARNING! THERE MAY BE SINGULAR CONFIGURATIONS!
    
    % delta feet size
    delta_x   =  (Config.feet_size(2,2) - Config.feet_size(2,1))/2;
    delta_x_0 = -(Config.feet_size(2,2) + Config.feet_size(2,1))/2;
    delta_y   =  (Config.feet_size(1,2) - Config.feet_size(1,1))/2;
    delta_y_0 =  (Config.feet_size(1,2) + Config.feet_size(1,1))/2;
    
    % parametrization: planar forces
    A = [Config.forceFrictionCoefficient*wrench(3).^2,                  -wrench(1)^2;
                       -wrench(2)^2,                Config.forceFrictionCoefficient*wrench(3).^2];
    
    tanh_square_xi_1_2 = (eye(2)/A)*[wrench(1)^2; wrench(2)^2];
    
    % INVERSE PARAMETRIZATION: CHECK TO AVOID SINGULARITIES
    if tanh_square_xi_1_2(1) < 0
        
        tanh_square_xi_1_2(1) = 0;
    end
    if tanh_square_xi_1_2(2) < 0
        
        tanh_square_xi_1_2(2) = 0;
    end
    
    inverseMappingForce_x = sign(wrench(1))*sqrt(tanh_square_xi_1_2(1));
    inverseMappingForce_y = sign(wrench(2))*sqrt(tanh_square_xi_1_2(2));
    
    if inverseMappingForce_x >= 1
        
        inverseMappingForce_x = 0.9;
    end
    if inverseMappingForce_y >= 1
        
        inverseMappingForce_y = 0.9;
    end
    if inverseMappingForce_x <= -1
        
        inverseMappingForce_x = -0.9;
    end
    if inverseMappingForce_y <= -1
        
        inverseMappingForce_y = -0.9;
    end
    
    xi_1 = atanh(inverseMappingForce_x);
    xi_2 = atanh(inverseMappingForce_y);
    
    % parametrization: vertical force
    
    % INVERSE PARAMETRIZATION: CHECK TO AVOID SINGULARITIES
    inverseMappingVerticalForce = wrench(3) - Config.fZmin;
    
    if inverseMappingVerticalForce < 1
        
        inverseMappingVerticalForce = 1;
    end
        
    xi_3 = log(inverseMappingVerticalForce);
    
    % parametrization: moments
    
    % INVERSE PARAMETRIZATION: CHECK TO AVOID SINGULARITIES
    inverseMappingMoment_x = (wrench(4) - delta_x_0*wrench(3))/(delta_x*wrench(3));
    inverseMappingMoment_y = (wrench(5) - delta_y_0*wrench(3))/(delta_y*wrench(3));
    inverseMappingMoment_z =  wrench(6)/(Config.torsionalFrictionCoefficient*wrench(3));
    
    if inverseMappingMoment_x >= 1
        
        inverseMappingMoment_x = 0.9;
        
    elseif inverseMappingMoment_x <= -1
        
        inverseMappingMoment_x = -0.9;
    end
    if inverseMappingMoment_y >= 1
        
        inverseMappingMoment_y = 0.9;
        
    elseif inverseMappingMoment_y <= -1
        
        inverseMappingMoment_y = -0.9;
    end
    if inverseMappingMoment_z >= 1
        
        inverseMappingMoment_z = 0.9;
        
    elseif inverseMappingMoment_z <= -1
        
        inverseMappingMoment_z = -0.9;
    end
        
    xi_4 = atanh(inverseMappingMoment_x);
    xi_5 = atanh(inverseMappingMoment_y);
    xi_6 = atanh(inverseMappingMoment_z);
    
    % parametrization vector
    xi_notSaturated = [xi_1; xi_2; xi_3; xi_4; xi_5; xi_6];
    
    % INVERSE PARAMETRIZATION: SATURATION
    xi = saturateInput(xi_notSaturated, [1.5,1.5,57.5,1.5,1.5,1.5], [-1.5,-1.5,0,-1.5,-1.5,-1.5]);
end
