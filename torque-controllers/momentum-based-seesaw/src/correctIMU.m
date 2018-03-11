function imu_H_imuAssumingNeckToZero = correctIMU(neckJoints)
% correctIMU Corrects the IMU transform computed assuming the neck joints
% equal to 0, return the imu_H_imuAssumingNeckToZero transform. 
% neckJoints is the vector of joints ( neck_pitch , neck_roll,
% neck_yaw) (0,1,2) of head part, expressed in radians (while on the port
% they are published in degree)

% Compute the imuAssumingNeckToZero_H_neckBase transform 
G_34=evalDHMatrix(9.5*1e-3,       0,        pi/2,  + pi/2);
G_45=evalDHMatrix(   0,           0,       -pi/2,  - pi/2);
G_56=evalDHMatrix(18.5*1e-3,  110.8*1e-3,  -pi/2,  + pi/2); 
G_6I=evalDHMatrix(   0,       6.6*1e-3,     pi/2,    0); 

imuAssumingNeckToZero_H_neckBase = G_34*G_45*G_56*G_6I;

% Compute the imu_H_neckBase transform 
G_34=evalDHMatrix(9.5*1e-3,      0,         pi/2,     neckJoints(1) + pi/2);
G_45=evalDHMatrix(   0,          0,        -pi/2,     neckJoints(2) - pi/2);
G_56=evalDHMatrix(18.5*1e-3,  110.8*1e-3,  -pi/2,     neckJoints(3) + pi/2); 
G_6I=evalDHMatrix(   0,       6.6*1e-3,     pi/2,     0); 

imu_H_neckBase = G_34*G_45*G_56*G_6I;

imu_H_imuAssumingNeckToZero = imu_H_neckBase/(imuAssumingNeckToZero_H_neckBase);

end

