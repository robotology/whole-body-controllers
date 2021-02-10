classdef casadi_block < matlab.System & matlab.system.mixin.Propagates
    % Casadi Implementation of the Momentum Based Velocity COntrol. 
    
    properties
        % Public, tunable properties.
        
    end
    
    properties (DiscreteState)
    end
    
    properties (Access = private)
       casadi_optimizer;
        M;
        h;
        Jc;
        Jcmm;
        nu_k;
        C_friction;
        b_friction;
        s_dot_star;
        H_star;
        tau_k;
        s_dot_k_1;
        v_b_k_1;
        f_k;
        Jc_dot_nu;
        sol;
        tau_meas;
        f_meas;
        solver_fails_counter;
        s;
        CoM_measured;
        K_friction;
        t_step;
        epsilon_CoM;
        CoM_max;
        CoM_min;
        CoM_vel_max;
        CoM_vel_min;
        
    end
    
    methods (Access = protected)
        function num = getNumInputsImpl(~)
            num = 31;
        end
        function num = getNumOutputsImpl(~)
            num = 4;
        end
        function [dt1,dt2,dt3,dt4] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
            dt4 = 'double';
            
        end
        
        function [dt1,dt2,dt3,dt4,dt5,dt6,dt7,dt8,dt9,dt10,dt11,dt12,dt13,dt14,dt15,dt16,dt17,dt18,dt19,dt20,dt21,dt22,dt23,dt24, ...
                dt25,dt26,dt27,dt28,dt29,dt30,dt31] = getInputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
            dt4 = 'double';
            dt5 = 'double';
            dt6 = 'double';
            dt7 = 'double';
            dt8 = 'double';
            dt9 = 'double';
            dt10 = 'double';
            dt11 = 'double';
            dt12 = 'double';
            dt13 = 'double';
            dt14 = 'double';
            dt15 = 'double';
            dt16 = 'double';
            dt17 = 'double';
            dt18 = 'double';
            dt19 = 'double';
            dt20 = 'double';
            dt21 = 'double';
            dt22 = 'double';
            dt23 = 'double';
            dt24 = 'double';
            dt25 = 'double';
            dt26 = 'double';
            dt27 = 'double';
            dt28 = 'double';
            dt29 = 'double';
            dt30 = 'double';
            dt31 = 'double';
        end
        function [sz1,sz2,sz3,sz4] = getOutputSizeImpl(obj)
            % Hack for avoiding hard coding the DoF of the Robot 
            size_temp = propagatedInputSize(obj,1);
            sz1 = [6,1];
            sz2 = [size_temp(1)-6,1];
            sz3 = [size_temp(1)-6,1];
            sz4 = [12,1];
        end
        function [sz1,sz2,sz3,sz4,sz5,sz6,sz7,sz8,sz9,sz10,sz11,sz12,sz13,sz14,sz15,sz16, sz17,sz18,sz19,sz20,sz21,sz22, ...
                sz23,sz24,sz25,sz26,sz27,sz28,sz29,sz30,sz31] = getInputSizeImpl(~)
            sz1 = [1,1];
            sz2 = [1,1];
            sz3 = [1,1];
            sz4 = [1,1];
            sz5 = [1,1];
            sz6 = [1,1];
            sz7 = [1,1];
            sz8 = [1,1];
            sz9 = [1,1];
            sz10 = [1,1];
            sz11 = [1,1];
            sz12 = [1,1];
            sz13 = [1,1];
            sz14 = [1,1];
            sz15 = [1,1];
            sz16 = [1,1];
            sz17 = [1,1];
            sz18 = [1,1];
            sz19 = [1,1];
            sz20 = [1,1];
            sz21 = [1,1];
            sz22 = [1,1];
            sz23 = [1,1];
            sz24 = [1,1];
            sz25 = [1,1];
            sz26 = [1,1];
            sz27 = [1,1];
            sz28 = [1,1];
            sz29 = [1,1];
            sz30 = [1,1];
            sz31 = [1,1];
            
        end
        function [cp1,cp2,cp3,cp4,cp5,cp6,cp7,cp8,cp9,cp10,cp11,cp12,cp13, cp14, cp15,cp16, cp17,cp18,cp19,cp20,cp21,...
                cp22,cp23,cp24,cp25,cp26,cp27,cp28,cp29,cp30,cp31] = isInputComplexImpl(~)
            cp1 = false;
            cp2 = false;
            cp3 = false;
            cp4 = false;
            cp5 = false;
            cp6 = false;
            cp7 = false;
            cp8 = false;
            cp9 = false;
            cp10 = false;
            cp11 = false;
            cp12 = false;
            cp13 = false;
            cp14 = false;
            cp15 = false;
            cp16 = false;
            cp17 = false;
            cp18 = false;
            cp19 = false;
            cp20 = false;
            cp21 = false;
            cp22 = false;
            cp23 = false;
            cp24 = false;
            cp25 = false;
            cp26 = false;
            cp27 = false;
            cp28 = false;
            cp29 = false;
            cp30 = false;
            cp31 = false;
        end
        function [cp1,cp2,cp3,cp4] = isOutputComplexImpl(~)
            cp1 = false;
            cp2 = false;
            cp3 = false;
            cp4 = false;
        end
        function [fz1,fz2,fz3,fz4,fz5,fz6,fz7,fz8,fz9,fz10, fz11,fz12,fz13,fz14,fz15,fz16, fz17,fz18,fz19,fz20,fz21, ...
                fz22,fz23,fz24,fz25,fz26,fz27,fz28,fz29,fz30,fz31] = isInputFixedSizeImpl(~)
            fz1 = true;
            fz2 = true;
            fz3 = true;
            fz4 = true;
            fz5 = true;
            fz6 = true;
            fz7 = true;
            fz8 = true;
            fz9 = true;
            fz10 = true;
            fz11 = true;
            fz12 = true;
            fz13 = true;
            fz14 = true;
            fz15 = true;
            fz16 = true;
            fz17 = true;
            fz18 = true;
            fz19 = true;
            fz20 = true;
            fz21 = true;
            fz22 = true;
            fz23 = true;
            fz24 = true;
            fz25 = true;
            fz26 = true;
            fz27 = true;
            fz28 = true;
            fz29 = true;
            fz30 = true;
            fz31 = true;
        end
        function [fz1,fz2,fz3,fz4] = isOutputFixedSizeImpl(~)
            fz1 = true;
            fz2 = true;
            fz3 = true;
            fz4 = true;
        end
        function setupImpl(obj,M,h,Jc,Jcmm,nu_k,Jc_dot_nu,C_friction,b_friction,H_star,s_dot_star,tau_meas, contact_status, f_meas, x_com, J_com, w_H_b,...
                s,CoM_measured,ADD_FRICTION, K_viscous_friction, T, Gamma, baseVelocitySat, jointsVelocitySat, torqueSat, wrenchesSat, tStep, NDOF, epsilon_CoM, CoM_limits,CoM_vel_limits)
            
            NDOF = size(K_viscous_friction,1);
            import casadi.*
            obj.casadi_optimizer = casadi.Opti();
            obj.solver_fails_counter = 0;
            
            % Setting the solver for the optimization problem
            options = struct;
            options.qpsol = 'qpoases';
            options.print_header = false;
            options.qpsol_options.print_problem = false;
            options.qpsol_options.print_problem = false;
            options.qpsol_options.print_time = false;
            options.qpsol_options.sparse = true;
            options.print_status = false;
            options.qpsol_options.error_on_fail= false;
            obj.casadi_optimizer.solver('sqpmethod', options);
            
            
            %Definition of the decision variables
            obj.tau_k = obj.casadi_optimizer.variable(NDOF);
            obj.s_dot_k_1 = obj.casadi_optimizer.variable(NDOF);
            obj.v_b_k_1 = obj.casadi_optimizer.variable(6);
            obj.f_k = obj.casadi_optimizer.variable(12);
            nu_k_1 = [obj.v_b_k_1; obj.s_dot_k_1];
            
            % Setting the parameter for the optimization solver
            obj.M = obj.casadi_optimizer.parameter(NDOF+6,NDOF+6);
            obj.h = obj.casadi_optimizer.parameter(NDOF+6);
            obj.Jc = obj.casadi_optimizer.parameter(12,NDOF+6);
            obj.Jcmm = obj.casadi_optimizer.parameter(6,NDOF+6);
            obj.nu_k = obj.casadi_optimizer.parameter(NDOF+6);
            obj.C_friction = obj.casadi_optimizer.parameter(38,12);
            obj.b_friction = obj.casadi_optimizer.parameter(38);
            obj.s_dot_star = obj.casadi_optimizer.parameter(NDOF);
            obj.H_star = obj.casadi_optimizer.parameter(6);
            obj.Jc_dot_nu = obj.casadi_optimizer.parameter(12);
            obj.tau_meas = obj.casadi_optimizer.parameter(NDOF);
            obj.f_meas = obj.casadi_optimizer.parameter(12);
            obj.s = obj.casadi_optimizer.parameter(NDOF);
            obj.CoM_measured = obj.casadi_optimizer.parameter(3);
            obj.K_friction = obj.casadi_optimizer.parameter(NDOF+6,NDOF+6);
            obj.t_step = obj.casadi_optimizer.parameter(1);
            obj.CoM_max = obj.casadi_optimizer.parameter(2);
            obj.CoM_min = obj.casadi_optimizer.parameter(2);
            obj.CoM_vel_max = obj.casadi_optimizer.parameter(2);
            obj.CoM_vel_min = obj.casadi_optimizer.parameter(2);
            obj.epsilon_CoM = obj.casadi_optimizer.parameter(2);
            
            
            %Weigth
            Weigth.PosturalTask    = 10;
            Weigth.MomentumLinear  = 2.0;
            Weigth.MomentumAngular = 30.00;
            Weigth.RegTorques      = 0.001;
            Weigth.RegVelocities   = (0.01)/10;
            Weigth.Wrenches        = 0.0*5.;
            
            % Selector Matrix
            
            B           = [ zeros(6,NDOF);  ...
                eye(NDOF,NDOF)];
            
            % Setting the objective function
            obj.casadi_optimizer.minimize(Weigth.RegTorques*sumsqr(obj.tau_k - obj.tau_meas)+                     ... % Torque Regularization
                Weigth.RegVelocities*sumsqr(nu_k_1(1:6))+                               ... % Velocity Regularization
                Weigth.PosturalTask*sumsqr(obj.s_dot_k_1-obj.s_dot_star)+               ...Postural Task
                Weigth.MomentumAngular*sumsqr(obj.H_star(4:6)-obj.Jcmm(4:6,:)*nu_k_1)); ... % Angular Momentum
                %Weigth.MomentumLinear*sumsqr(obj.H_star(1:3)-obj.Jcmm(1:3,:)*nu_k_1));  ... % Linear Momentum
            
            
            % Setting Constraint
            % Dynamics
            obj.casadi_optimizer.subject_to((obj.M/obj.t_step + obj.K_friction)*(nu_k_1)-(obj.M*obj.nu_k)/obj.t_step+obj.h== B*obj.tau_k +obj.Jc'*obj.f_k);
            % Holonomic Constraint
            % acceleration-wise
            % obj.casadi_optimizer.subject_to(obj.Jc*(nu_k_1-obj.nu_k)/tStep==-obj.Jc_dot_nu);
            % velocity-wise
            obj.casadi_optimizer.subject_to(obj.Jc*(nu_k_1)==zeros(12,1));
            % Friction Cones
            obj.casadi_optimizer.subject_to(obj.C_friction*obj.f_k<obj.b_friction);
            % Angular Momentum
            %obj.casadi_optimizer.subject_to(obj.H_star(4:6,:)==obj.Jcmm(4:6,:)*nu_k_1);
            
            % Constraint on the CoM Velocity
            obj.casadi_optimizer.subject_to(obj.Jcmm(1,:)*nu_k_1<(tanh(obj.epsilon_CoM*(obj.CoM_max(1)-obj.CoM_measured(1)))*obj.CoM_vel_max(1)));
            obj.casadi_optimizer.subject_to(obj.Jcmm(1,:)*nu_k_1>(tanh(obj.epsilon_CoM*(obj.CoM_measured(1)-obj.CoM_min(1)))*obj.CoM_vel_min(1)));
            obj.casadi_optimizer.subject_to(obj.Jcmm(2,:)*nu_k_1<(tanh(obj.epsilon_CoM*(obj.CoM_max(2)-obj.CoM_measured(2)))*obj.CoM_vel_max(2)));
            obj.casadi_optimizer.subject_to(obj.Jcmm(2,:)*nu_k_1>(tanh(obj.epsilon_CoM*(obj.CoM_measured(2)-obj.CoM_min(2)))*obj.CoM_vel_min(2)));
            
            %THE BOUND FOR NOW ARE DE-ACTIVATED
            % Setting upper and lower bound
            % Lower and Upper Bound NOTE by using a casadi optimizer object, we loose the capability of interpreting the lower and
            % uppe bound in a smart way.
            % obj.casadi_optimizer.subject_to(Sat.BaseLinearVelocity_min<= obj.v_b_k_1(1:3)<=Sat.BaseLinearVelocity_max);
            % obj.casadi_optimizer.subject_to(Sat.BaseAngVelocity_min<= obj.v_b_k_1(4:end)<=Sat.BaseAngVelocity_max);
            % obj.casadi_optimizer.subject_to(Sat.JointsVelocity_min<= obj.s_dot_k_1<= Sat.JointsVelocity_max);
            % obj.casadi_optimizeru.sbject_to(Sat.torques_min(1:3)<= obj.tau_k(1:3)<= Sat.torques_max(1:3));
            
        end
        
        function [v_b_star,s_dot_k_1_star,tau_star,f_star] = stepImpl(obj,M,h,Jc,Jcmm,nu_k,Jc_dot_nu,C_friction,b_friction,H_star,s_dot_star,tau_meas, contact_status, f_meas, x_com, J_com, w_H_b,...
                s,CoM_measured,ADD_FRICTION, K_viscous_friction, T, Gamma, baseVelocitySat, jointsVelocitySat, torqueSat, wrenchesSat, tStep, NDOF,epsilon_CoM, CoM_limits,CoM_vel_limits)
            solver_succeded = true;
            
            % Compute Centroidayl Dynamics
            selector = zeros(length(nu_k),1);
            selector(3) = 1 ;
            DYNAMICS.M = M;
            DYNAMICS.h = h;
            DYNAMICS.g = M*selector*(9.81);
            DYNAMICS.Jc = Jc;
            DYNAMICS.dJc_nu = Jc_dot_nu;
            FORKINEMATICS.xCoM = x_com;
            vel_com = J_com*nu_k;
            FORKINEMATICS.dxCoM = vel_com(1:3);
            
            STATE.nu = nu_k;
            STATE.dx_b = nu_k(1:3);
            STATE.x_b = w_H_b(1:3,4);
            centroidalDyn = centroidalConversion(DYNAMICS,FORKINEMATICS,STATE);
            
            %K friction
            K_friction_new = zeros(NDOF+6);
            
            if(ADD_FRICTION)
                invTGamma                = eye(size(Gamma))/(T*Gamma);
                invTGamma_t              = eye(size(Gamma))/(transpose(T*Gamma));
                K_friction_new(7:end, 7:end) = invTGamma_t*K_viscous_friction*invTGamma;
            end
            
            % Update casadi optimizer parameters
            obj.casadi_optimizer.set_value(obj.M, centroidalDyn.M);
            obj.casadi_optimizer.set_value(obj.h,( centroidalDyn.C_nu+centroidalDyn.g));
            obj.casadi_optimizer.set_value(obj.Jc, centroidalDyn.Jc);
            obj.casadi_optimizer.set_value(obj.Jcmm, Jcmm);
            obj.casadi_optimizer.set_value(obj.nu_k, centroidalDyn.nu);
            obj.casadi_optimizer.set_value(obj.C_friction,C_friction);
            obj.casadi_optimizer.set_value(obj.b_friction,b_friction);
            obj.casadi_optimizer.set_value(obj.s_dot_star,s_dot_star);
            obj.casadi_optimizer.set_value(obj.H_star,H_star);
            obj.casadi_optimizer.set_value(obj.Jc_dot_nu, centroidalDyn.dJc_nu);
            obj.casadi_optimizer.set_value(obj.tau_meas, tau_meas);
            obj.casadi_optimizer.set_value(obj.f_meas, f_meas);
            obj.casadi_optimizer.set_value(obj.s,s);
            obj.casadi_optimizer.set_value(obj.CoM_measured,CoM_measured)
            obj.casadi_optimizer.set_value(obj.K_friction,K_friction_new);
            obj.casadi_optimizer.set_value(obj.t_step,tStep);
            obj.casadi_optimizer.set_value(obj.CoM_max, CoM_limits(:,2));
            obj.casadi_optimizer.set_value(obj.CoM_min, CoM_limits(:,1));
            obj.casadi_optimizer.set_value(obj.CoM_vel_max, CoM_vel_limits(:,2));
            obj.casadi_optimizer.set_value(obj.CoM_vel_min, CoM_vel_limits(:,1));
            obj.casadi_optimizer.set_value(obj.epsilon_CoM, epsilon_CoM);
            
            % Computing the solution
            try
                obj.sol = obj.casadi_optimizer.solve; % could raise an error
                v_b_star = full(obj.sol.value(obj.v_b_k_1));
                s_dot_k_1_star = full(obj.sol.value(obj.s_dot_k_1));
                tau_star = full(obj.sol.value(obj.tau_k));
                f_star = full(obj.sol.value(obj.f_k));
                obj.casadi_optimizer.set_initial(obj.casadi_optimizer.x, obj.sol.value(obj.casadi_optimizer.x));
            catch exception
                try
                    solver_succeded = false;
                    obj.casadi_optimizer.debug.show_infeasibilities;
                    disp(exception.getReport)
                catch
                    disp(exception.message)
                end
            end
            
            
            % If the solver did not succed, put to zero the control input
            % and increase the related counter
            if(~solver_succeded)
                obj.solver_fails_counter = obj.solver_fails_counter +1;
                v_b_star = zeros(6,1);
                s_dot_k_1_star = zeros(26,1);
                tau_star = zeros(26,1);
                f_star = zeros(12,1);
                if(obj.solver_fails_counter>4)
                    error('solver failed more that 4 times');
                end
            else
                obj.solver_fails_counter = 0;
                
            end
            
        end
        
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end
