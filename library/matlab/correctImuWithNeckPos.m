% CORRECTIMUWITHNECKPOS corrects the IMU transform computed assuming 
%                       the neck joint positions equal to 0, and returns
%                       the imu_H_imuAssumingNeckToZero transform. 
%                           
% FORMAT: imu_H_imuAssumingNeckToZero = correctImuWithNeckPos(neckJoints)    
%
% INPUT:  - neckJoints = [3 * 1] vector of joints (neck_pitch, neck_roll,
%                         neck_yaw) expressed in radians (while on the port 
%                         they are published in degrees)
%
% OUTPUT: - imu_H_imuAssumingNeckToZero= [4 * 4] imu to corrected_imu transform
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function imu_H_imuAssumingNeckToZero = correctImuWithNeckPos(neckJoints)

    % Compute the imuAssumingNeckToZero_H_neckBase transform 
    H_34 = evalDHMatrix( 9.5*1e-3,          0,  pi/2, +pi/2);
    H_45 = evalDHMatrix(        0,          0, -pi/2, -pi/2);
    H_56 = evalDHMatrix(18.5*1e-3, 110.8*1e-3, -pi/2, +pi/2); 
    H_61 = evalDHMatrix(        0,   6.6*1e-3,  pi/2,     0); 

    imuAssumingNeckToZero_H_neckBase = H_34 * H_45 * H_56 * H_61;

    % Compute the imu_H_neckBase transform 
    H_34 = evalDHMatrix( 9.5*1e-3,          0,  pi/2, neckJoints(1) +pi/2);
    H_45 = evalDHMatrix(        0,          0, -pi/2, neckJoints(2) -pi/2);
    H_56 = evalDHMatrix(18.5*1e-3, 110.8*1e-3, -pi/2, neckJoints(3) +pi/2); 
    H_61 = evalDHMatrix(        0,   6.6*1e-3,  pi/2,                   0); 

    imu_H_neckBase = H_34 * H_45 * H_56 * H_61;

    imu_H_imuAssumingNeckToZero = imu_H_neckBase/(imuAssumingNeckToZero_H_neckBase);
end
