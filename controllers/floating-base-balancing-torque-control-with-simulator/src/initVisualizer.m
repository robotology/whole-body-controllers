%% configuration for the matlab iDyntree visualizer

confVisualizer.fileName = robot_config.fileName;
confVisualizer.meshFilePrefix = robot_config.meshFilePrefix;
confVisualizer.jointOrder = robot_config.jointOrder;
confVisualizer.robotFrames = robot_config.robotFrames;

% initial joints configuration specified in configRobot
confVisualizer.joints_positions = robot_config.initialConditions.s;
confVisualizer.world_H_base = robot_config.initialConditions.w_H_b;

% size of the square you see around the robot
confVisualizer.aroundRobot = visualizerAroundRobot;

% refresh rate of the picure
confVisualizer.tStep = 0.040; % here equal to the time step used in the simulink model
