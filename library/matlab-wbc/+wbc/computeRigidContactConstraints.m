function [ConstraintsMatrix, bVectorConstraints] = computeRigidContactConstraints(staticFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient, contactAreaSize, fZmin)

    % COMPUTERIGIDCONTACTCONSTRAINTS computes the constraint matrix and bias vector
    %                                for applying friction cones, unilateral
    %                                constraints and local CoP constraints at
    %                                contact locations (assuming rigid contacts).
    %                                The output are the matrix C and the
    %                                bias vector b such that:
    %
    %                                    C*f < b
    %
    %                                f are the contact forces and moments
    %                                w.r.t. a reference frames attached to
    %                                the contact location.
    %
    % FORMAT: [ConstraintsMatrix, bVectorConstraints] = computeRigidContactConstraints ...
    %             (staticFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient, footSize, fZmin)
    %
    % INPUT:   - staticFrictionCoefficient    = static linear coefficient of friction
    %          - numberOfPoints               = number of points in each quadrants for
    %                                           linearizing friction cone
    %          - torsionalFrictionCoefficient = torsional coefficient of friction
    %          - contactAreaSize              = physical size of the contact area
    %          - fZmin                        = minimal positive vertical force at contact
    %
    % OUTPUT:  - ConstraintsMatrix  = constraint matrix
    %          - bVectorConstraints = bias vector constraints
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % Compute friction cones contraints approximation with straight lines

    % split the pi/2 angle into numberOfPoints -1
    segmentAngle           = pi/2 / (numberOfPoints - 1);

    % define angle
    angle                  = 0:segmentAngle:(2*pi -segmentAngle);
    points                 = [cos(angle); sin(angle)];
    numberOfEquations      = size(points, 2);
    assert(size(points, 2) == (4 * (numberOfPoints - 2) + 4));

    % A_ineq*x <= b, with b all zeros
    A_ineq                 = zeros(numberOfEquations, 6);

    % define equations
    for i = 1 : numberOfEquations

        firstPoint  = points(:, i);
        secondPoint = points(:, rem(i, numberOfEquations) + 1);

        % define line passing through the above points
        angularCoefficients = (secondPoint(2) -firstPoint(2)) / (secondPoint(1) -firstPoint(1));

        offsets             = firstPoint(2) -angularCoefficients*firstPoint(1);

        inequalityFactor    = + 1;

        % if any of the two points are between pi and 2pi, then the inequality is
        % in the form of y >= m*x + q, and is needed to change the sign of it.
        if (angle(i) > pi || angle(rem(i, numberOfEquations) + 1) > pi)

            inequalityFactor = -1;
        end

        % a wrench is 6 dimensional f = [fx, fy, fz, mux, muy, muz]'
        % there are constraints on fy and fz, and the offset will be multiplied
        % by mu * fx
        %
        A_ineq(i,:) = inequalityFactor.* [-angularCoefficients, 1, (-offsets*staticFrictionCoefficient), 0, 0, 0];
    end

    %% POSITIVITY OF VERTICAL FORCE, AND COP
    %
    %  Positivity of vertical force: F_z > fZmin
    %  Center of pressure (CoP):  dimMinArea_y <  torque_x/F_z < dimMaxArea_y
    %                             dimMinArea_x < -torque_y/F_z < dimMaxArea_x
    %
    %  footSize  = [dimMiArea_x,  dimMaxArea_x
    %               dimMinArea_y, dimMaxArea_y]
    %
    %                    F_x   F_y                           F_z     torque_x     torque_y      torque_z
    ConstraintsMatrix = [ 0,    0, -torsionalFrictionCoefficient,           0,           0,            1;  %  torque_z -torsionalFrictionCoefficient*F_z < 0
        0,    0, -torsionalFrictionCoefficient,           0,           0,           -1;  % -torque_z -torsionalFrictionCoefficient*F_z < 0
        0,    0,                            -1,           0,           0,            0;  % -F_z                                        < -fZmin
        0,    0,          contactAreaSize(1,1),           0,           1,            0;  %  torque_y +dimMinArea_x*F_z                 < 0
        0,    0,         -contactAreaSize(1,2),           0,          -1,            0;  % -torque_y -dimMaxArea_x*F_z                 < 0
        0,    0,          contactAreaSize(2,1),          -1,           0,            0;  % -torque_x +dimMinArea_y*F_z                 < 0
        0,    0,         -contactAreaSize(2,2),           1,           0,            0]; %  torque_x -dimMaxArea_y*F_z                 < 0

    % add inequality constraints
    ConstraintsMatrix  =  [A_ineq;
        ConstraintsMatrix ];

    bVectorConstraints = [zeros(size(A_ineq,1), 1); zeros(7,1)];

    bVectorConstraints(3 + size(A_ineq,1)) = -fZmin;
end
