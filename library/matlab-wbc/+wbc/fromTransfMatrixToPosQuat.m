function q = fromTransfMatrixToPosQuat(H)

    % FROMTRANSFMATRIXTOQUATERNIONS computes a link pose using position 
    %                               + quaternion rapresentation. The input is 
    %                               the link pose represented by a transformation matrix.
    %
    % FORMAT: q = fromTransfMatrixToPosQuat(H)  
    %
    % INPUT:  - H = [4 * 4] transf matrix representing link pose
    %
    % OUTPUT: - q = [7 * 1] position + quaternions representing link pose
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % Separate the rotation matrix
    R       = H(1:3,1:3);
    qt_b    = zeros(4,1);
    
    % min. value to treat a number as zero
    epsilon = 1e-12; 
    
    % Compute the corresponding (unit) quaternion from a given rotation matrix R:
    %
    % The transformation uses the computational efficient algorithm of Stanley.
    % To be numerically robust, the code determines the set with the maximum
    % divisor for the calculation.
    %
    % For further details about the Stanley Algorithm, see:
    %   [1] Optimal Spacecraft Rotational Maneuvers, John L. Junkins & James D. Turner, Elsevier, 1986, pp. 28-29, eq. (2.57)-(2.59).
    %   [2] Theory of Applied Robotics: Kinematics, Dynamics, and Control, Reza N. Jazar, 2nd Edition, Springer, 2010, p. 110, eq. (3.149)-(3.152).
    % Note: There exist also an optimized version of the Stanley method and is the fastest
    %       possible computation method for Matlab, but it does not cover all special cases.
    % Further details about the fast calculation can be found at:
    %   [3] Modelling and Control of Robot Manipulators, L. Sciavicco & B. Siciliano, 2nd Edition, Springer, 2008,
    %       p. 36, formula (2.30).
    %
    tr = R(1,1) + R(2,2) + R(3,3);
    
    if (tr > epsilon)
        
            % scalar part:
            qt_b(1,1) = 0.5*sqrt(tr + 1);
            s_inv     = 1/(qt_b(1,1)*4);
            
            % vector part:
            qt_b(2,1) = (R(3,2) - R(2,3))*s_inv;
            qt_b(3,1) = (R(1,3) - R(3,1))*s_inv;
            qt_b(4,1) = (R(2,1) - R(1,2))*s_inv;
    else
        % if tr <= 0, find the greatest diagonal element for calculating
        % the scale factor s and the vector part of the quaternion
        if ((R(1,1) > R(2,2)) && (R(1,1) > R(3,3)))
            
            qt_b(2,1) = 0.5*sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
            s_inv     = 1/(qt_b(2,1)*4);

            qt_b(1,1) = (R(3,2) + R(2,3))*s_inv;
            qt_b(3,1) = (R(2,1) + R(1,2))*s_inv;
            qt_b(4,1) = (R(3,1) + R(1,3))*s_inv;
            
        elseif (R(2,2) > R(3,3))
            
            qt_b(3,1) = 0.5*sqrt(R(2,2) - R(3,3) - R(1,1) + 1);
            s_inv     = 1/(qt_b(3,1)*4);

            qt_b(1,1) = (R(1,3) - R(3,1))*s_inv;
            qt_b(2,1) = (R(2,1) + R(1,2))*s_inv;
            qt_b(4,1) = (R(3,2) + R(2,3))*s_inv;
        else
            qt_b(4,1) = 0.5*sqrt(R(3,3) - R(1,1) - R(2,2) + 1);
            s_inv     = 1/(qt_b(4,1)*4);
            
            qt_b(1,1) = (R(2,1) - R(1,2))*s_inv;
            qt_b(2,1) = (R(3,1) + R(1,3))*s_inv;
            qt_b(3,1) = (R(3,2) + R(2,3))*s_inv;
        end
    end

    % Final state converted into quaternion rapresentation
    q = [H(1:3,4); qt_b];
end