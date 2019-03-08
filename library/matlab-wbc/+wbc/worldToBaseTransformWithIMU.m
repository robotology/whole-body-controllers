function [w_H_b, wImu_H_base, wImu_H_fixedLink_0] = worldToBaseTransformWithIMU(imu_H_fixedLink, imu_H_fixedLink_0, fixedLink_H_base, rpyFromIMU_0, rpyFromIMU, FILTER_IMU_YAW)

    % WORLDTOBASETRANSFORMWITHIMU calculates the world-to-base frame transformation
    %                             matrix. The orientation is updated by 
    %                             using informations from IMU.
    %
    % FORMAT: [w_H_b, wImu_H_base, wImu_H_fixedLink_0] = worldToBaseTransformWithIMU(imu_H_fixedLink, imu_H_fixedLink_0, fixedLink_H_base, rpyFromIMU_0, rpyFormIMU, FILTER_IMU_YAW)
    %
    % INPUT:  - imu_H_fixedLink   = [4 * 4] imu to fixed link transform
    %         - imu_H_fixedLink_0 = [4 * 4] imu to fixed link transform at t = 0
    %         - fixedLink_H_base  = [4 * 4] fixed link to base transform
    %         - rpyFromIMU_0      = IMU orientation at t = 0
    %         - rpyFromIMU        = IMU orientation
    %         - FILTER_IMU_YAW    = boolean
    %
    % OUTPUT: - w_H_b = [4 * 4] world to base frame transformation matrix
    %         - wImu_H_base = [4 * 4] IMU inertial frame to base frame
    %                         transformation matrix
    %         - wImu_H_fixedLink_0 = [4 * 4] IMU inertial frame to fixed frame
    %                                        transformation matrix at t = 0.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---

    % WARNING!!! Converting the inertial values from grad into rad
    rpyFromIMU         = (rpyFromIMU   * pi)/180;
    rpyFromIMU_0       = (rpyFromIMU_0 * pi)/180;

    % Composing the rotation matrix:
    % See http://wiki.icub.org/images/8/82/XsensMtx.pdf page 12
    wImu_R_imu         = wbc.rotz(rpyFromIMU(3))*wbc.roty(rpyFromIMU(2))*wbc.rotx(rpyFromIMU(1));
    wImu_R_imu_0       = wbc.rotz(rpyFromIMU_0(3))*wbc.roty(rpyFromIMU_0(2))*wbc.rotx(rpyFromIMU_0(1));

    % Rotation between the IMU and the fixed link
    imu_R_fixedLink    = imu_H_fixedLink(1:3,1:3);
    imu_R_fixedLink_0  = imu_H_fixedLink_0(1:3,1:3);

    % Rotation between the IMU inertial frame and the fixed link
    wImu_R_fixedLink   = wImu_R_imu * imu_R_fixedLink;
    wImu_R_fixedLink_0 = wImu_R_imu_0 * imu_R_fixedLink_0;

    % Convert into roll-pitch-yaw
    rollPitchYaw_fixedLink_0 = wbc.rollPitchYawFromRotation(wImu_R_fixedLink_0);
    rollPitchYaw_fixedLink   = wbc.rollPitchYawFromRotation(wImu_R_fixedLink);

    % Filter the Yaw angle (may be measured wrong by the IMU)
    rollPitchYaw_filtered    = rollPitchYaw_fixedLink;

    if FILTER_IMU_YAW
        
        rollPitchYaw_filtered(3) = rollPitchYaw_fixedLink_0(3);
    end

    wImu_R_fixedLink   = wbc.rotz(rollPitchYaw_filtered(3))*wbc.roty(rollPitchYaw_filtered(2))*wbc.rotx(rollPitchYaw_filtered(1));

    % IMU inertial frame to fixed link transform
    wImu_H_fixedLink   = [wImu_R_fixedLink, zeros(3,1)
                          zeros(1,3),       1     ];
          
    wImu_H_fixedLink_0 = [wImu_R_fixedLink_0, zeros(3,1)
                          zeros(1,3),         1     ];

    % IMU inertial frame to base link transform             
    wImu_H_base        = wImu_H_fixedLink * fixedLink_H_base;

    %% World to base frame transformation matrix
    w_H_b              = wImu_H_fixedLink_0\wImu_H_base;   
end