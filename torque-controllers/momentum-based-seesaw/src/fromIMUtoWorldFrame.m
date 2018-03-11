function [w_R_s,s_omega,orientFromIMURobot] = fromIMUtoWorldFrame ...
                                              (imu_H_fixedLink,imu_H_fixedLink_0,imuRotation_0,imuRotation,wImu_omega,neckPosition,CONFIG,seesaw)

% Transformation between the world and the fixed link at time 0 (assumption: the
% robot is in a known position on the seesaw at time 0);
w_H_fixedLink_0 = seesaw.w_H_fixedLink_0;

% Transformation between world and robot imu at time 0
w_H_imu_0 = w_H_fixedLink_0/imu_H_fixedLink_0;
w_R_imu_0 = w_H_imu_0(1:3,1:3);

% Converting the IMU angles from grad into rad
imuRot_rad = (imuRotation * pi)/180;
imuRot_rad_0 = (imuRotation_0 * pi)/180;

% Composing the rotation matrix:
% See http://wiki.icub.org/images/8/82/XsensMtx.pdf page 12
wImu_R_imu = rotz(imuRot_rad(3))*roty(imuRot_rad(2))*rotx(imuRot_rad(1));
wImu_R_imu_0 = rotz(imuRot_rad_0(3))*roty(imuRot_rad_0(2))*rotx(imuRot_rad_0(1));

% Relative rotation between wImu and the world frame (should be constant!)
w_R_wImu = w_R_imu_0/wImu_R_imu_0;

% Correcting neck movements
wImu_H_wImuAssumingNeckToZero = correctIMU(neckPosition);
wImu_R_imu = wImu_H_wImuAssumingNeckToZero(1:3,1:3) * wImu_R_imu;

% Rotation of the fixed link (= seesaw orintation) w.r.t. the imu world
imu_R_fixedlink = imu_H_fixedLink(1:3,1:3);
wImu_R_fixedLink = wImu_R_imu * imu_R_fixedlink;

% Finally, rotation of the seesaw w.r.t the world frame
w_R_s = w_R_wImu * wImu_R_fixedLink;

% Filter the pitch and yaw angle. Given the seesaw shape, they should be zero
rollPitchYaw_seesaw = rollPitchYawFromRotation(w_R_s);

if CONFIG.YAW_IMU_FILTER
    rollPitchYaw_seesaw(3) = 0;
end
if CONFIG.PITCH_IMU_FILTER
    rollPitchYaw_seesaw(2) = 0;
end

% In case it is required, add a constant offset to the measurements
rollPitchYaw_seesaw = rollPitchYaw_seesaw + seesaw.offset*pi/180;

w_R_s = rotz(rollPitchYaw_seesaw(3))*roty(rollPitchYaw_seesaw(2))*rotx(rollPitchYaw_seesaw(1));

% seesaw orientation (for comparison with the orientation given by the seesaw IMU
orientFromIMURobot = rollPitchYaw_seesaw;

% seesaw angular velocity in a frame attached to the seesaw
s_omega = transpose(w_R_s) * w_R_wImu * wImu_omega;

end
