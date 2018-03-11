function w_data = rotateImuWorldFrame(wImu_data,seesaw)

% the IMU mounted on the seesaw has its reference frame (wImu) oriented
% differently from the world reference frame (w). Therefore, this function
% project the acceleration/velocity of the seesaw from the wImu to the world
% frame.

% Relative rotation between the wImu (seesaw IMU) and the world frame
w_R_wImu = seesaw.w_R_wImu;

% Data in world frame
w_data = w_R_wImu * wImu_data;

end