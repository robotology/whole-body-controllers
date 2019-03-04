

Config.uDotMax

  
 nu_b = computeBaseVelocityWithFeetContact(J_l_sole, J_r_sole, feetContactStatus, jointVel, pinvDampTolerance)
        
        
        
        
        
        [w_H_b, wImu_H_base] = worldToBaseTransformWithIMU(imu_H_fixedLink, imu_H_fixedLink_0, fixedLink_H_base, rpyFromIMU_0, rpyFormIMU, FILTER_IMU_YAW)
           
        wImu_H_wImuAssumingNeckToZero = wbc.correctImuWithNeckPos(neck_pos);

    wImu_H_base  = wImu_H_wImuAssumingNeckToZero * wImu_H_base;
    
    imu_H_imuAssumingNeckToZero = correctIMUWithNeckPos(neckJointsPositions)