function imu_H_imuAssumingNeckToZero = correctIMUWithNeckPos(neckJointsPositions)

    % CORRECTIMUWITHNECKPOS to be used with the iCub HEAD IMU. It corrects
    %                       the IMU transformation matrix computed assuming
    %                       the neck joints position equals to 0, and returns
    %                       the imu_H_imuAssumingNeckToZero transform.
    %
    % FORMAT: imu_H_imuAssumingNeckToZero = correctIMUWithNeckPos(neckJointsPositions)
    %
    % INPUT:  - neckJointsPositions = [3 * 1] vector of joints (neck_pitch, neck_roll,
    %                                         neck_yaw) expressed in radians
    %
    % OUTPUT: - imu_H_imuAssumingNeckToZero = [4 * 4] imu to corrected_imu transform
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % Compute the imuAssumingNeckToZero_H_neckBase transform
    H_34 = evalDHMatrix( 9.5*1e-3,          0,  pi/2, +pi/2);
    H_45 = evalDHMatrix(        0,          0, -pi/2, -pi/2);
    H_56 = evalDHMatrix(18.5*1e-3, 110.8*1e-3, -pi/2, +pi/2);
    H_61 = evalDHMatrix(        0,   6.6*1e-3,  pi/2,     0);

    imuAssumingNeckToZero_H_neckBase = H_34*H_45*H_56*H_61;

    % Compute the imu_H_neckBase transform
    H_34 = evalDHMatrix( 9.5*1e-3,          0,  pi/2, neckJointsPositions(1) +pi/2);
    H_45 = evalDHMatrix(        0,          0, -pi/2, neckJointsPositions(2) -pi/2);
    H_56 = evalDHMatrix(18.5*1e-3, 110.8*1e-3, -pi/2, neckJointsPositions(3) +pi/2);
    H_61 = evalDHMatrix(        0,   6.6*1e-3,  pi/2,                   0);

    imu_H_neckBase = H_34*H_45*H_56*H_61;

    imu_H_imuAssumingNeckToZero = imu_H_neckBase/(imuAssumingNeckToZero_H_neckBase);
end

function H = evalDHMatrix(a, d, alpha, theta)

    % EVALDHMATRIX calculates the DH matrix from angles and parameters.
    %
    % FORMAT: H = evalDHMatrix(a, d, alpha, theta)
    %
    % INPUT:  - a = dimension (m)
    %         - d = dimension (m)
    %         - alpha = angle (rad)
    %         - theta = angle (rad)
    %
    % OUTPUT: - H = [4 * 4] DH matrix
    %

    H = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), cos(theta)*a
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), sin(theta)*a
        0,             sin(alpha),             cos(alpha),            d
        0,                      0,                      0,            1];
end
