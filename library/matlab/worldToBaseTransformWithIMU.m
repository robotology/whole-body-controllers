% WORLDTOBASETRANSFORMWITHIMU calculates the world-to-base transformation
%                             matrix using IMU orientation.
%
% FORMAT: w_H_b = worldToBaseTransformWithIMU(imu_H_link,imu_H_link_0,link_H_base,inertial_0,inertial,neck_pos,Config)    
%
% INPUT:  - imu_H_link = [4 * 4] imu to fixed link transform
%         - imu_H_link_0 = [4 * 4] imu to fixed link transform at 0
%         - link_H_base = [4 * 4] fixed link to base transform
%         - inertial_0 = IMU orientation, velocity, acceleration at 0
%         - inertial = IMU orientation, velocity, acceleration
%         - neck_pos = [3 * 1] neck position
%         - Config = user defined parameters
%
% OUTPUT: - w_H_b = [4 * 4] world to base transform
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function w_H_b = worldToBaseTransformWithIMU(imu_H_link,imu_H_link_0,link_H_base,inertial_0,inertial,neck_pos,Config)

    % Converting the inertial values from grad into rad
    inertial        = (inertial   * pi)/180;
    inertial_0      = (inertial_0 * pi)/180;

    % Composing the rotation matrix:
    % See http://wiki.icub.org/images/8/82/XsensMtx.pdf page 12
    wImu_R_imu      = rotz(inertial(3))*roty(inertial(2))*rotx(inertial(1));
    wImu_R_imu_0    = rotz(inertial_0(3))*roty(inertial_0(2))*rotx(inertial_0(1));

    % Rotation between the IMU and the fixed link
    imu_R_link      = imu_H_link(1:3,1:3);
    imu_R_link_0    = imu_H_link_0(1:3,1:3);

    % Rotation between the IMU inertial frame and the fixed link
    wImu_R_link     = wImu_R_imu * imu_R_link;
    wImu_R_link_0   = wImu_R_imu_0 * imu_R_link_0;

    % Convert into roll-pitch-yaw
    rollPitchYaw_link_0 = rollPitchYawFromRotation(wImu_R_link_0);
    rollPitchYaw_link   = rollPitchYawFromRotation(wImu_R_link);

    rollPitchYawFiltered_link = rollPitchYaw_link;

    if Config.FILTER_IMU_YAW
        rollPitchYawFiltered_link(3) = rollPitchYaw_link_0(3);
    end
    if Config.FILTER_IMU_PITCH
        rollPitchYawFiltered_link(2) = rollPitchYaw_link_0(2);
    end

    wImu_R_link    = rotz(rollPitchYawFiltered_link(3))*roty(rollPitchYawFiltered_link(2))*rotx(rollPitchYawFiltered_link(1));

    % IMU inertial frame to fixed link transform
    wImu_H_link    = [wImu_R_link,   zeros(3,1)
                      zeros(1,3),       1     ];
          
    wImu_H_link_0  = [wImu_R_link_0, zeros(3,1)
                      zeros(1,3),       1     ];

    % IMU inertial frame to base link transform             
    wImu_H_base    = wImu_H_link * link_H_base;

    %% Correct IMU with neck position
    wImu_H_wImuAssumingNeckToZero = correctImuWithNeckPos(neck_pos);

    wImu_H_base = wImu_H_wImuAssumingNeckToZero * wImu_H_base;
    w_H_b       = wImu_H_link_0\wImu_H_base;
    
end
