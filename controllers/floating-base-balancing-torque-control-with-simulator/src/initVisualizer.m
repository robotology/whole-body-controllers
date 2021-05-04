% configuration for the matlab iDyntree visualizer

%% Specific parameters

% Do you want to enable the Visualizer?
confVisualizer.visualizeRobot = true;

% size of the square you see around the robot
confVisualizer.aroundRobot = 1; % [m]

% refresh rate of the picure
confVisualizer.tStep = 0.010; % here equal to the time step used in the simulink model


%% Parameters copied from robot_config

% Robot description
confVisualizer.fileName = robot_config.fileName;
confVisualizer.meshFilePrefix = robot_config.meshFilePrefix;
confVisualizer.jointOrder = robot_config.jointOrder;
confVisualizer.robotFrames = robot_config.robotFrames;

% initial joints configuration specified in configRobot
confVisualizer.joints_positions = robot_config.initialConditions.s;
confVisualizer.world_H_base = robot_config.initialConditions.w_H_b;
