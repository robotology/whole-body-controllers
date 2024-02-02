%% Settings

robotName='iCubGenova04'; %% Name of the robot

testFrames = {'l_hand', 'r_hand', 'head'}; %% The frames to display

meshFilePrefix = [getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX') '/share']; %% Path to the model meshes

modelPath = [getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX') '/share/iCub/robots/' robotName '/'];  %% Path to the robot model

fileName='model.urdf'; %% Name of the urdf file

jointOrder={     %% The list and the order of the joints
    'r_hip_pitch';
    'r_hip_roll';
    'r_hip_yaw';
    'r_knee';
    'r_ankle_pitch';
    'r_ankle_roll';
    'l_hip_pitch';
    'l_hip_roll';
    'l_hip_yaw';
    'l_knee';
    'l_ankle_pitch';
    'l_ankle_roll';
    'torso_pitch';
    'torso_roll';
    'torso_yaw';
    'r_shoulder_pitch';
    'r_shoulder_roll';
    'r_shoulder_yaw';
    'r_elbow';
    'r_wrist_prosup';
    'r_wrist_pitch';
    'r_wrist_yaw';
    'l_shoulder_pitch';
    'l_shoulder_roll';
    'l_shoulder_yaw';
    'l_elbow';
    'l_wrist_prosup';
    'l_wrist_pitch';
    'l_wrist_yaw';
    'neck_pitch';
    'neck_roll';
    'neck_yaw'
    };

groundFrame = 'l_sole'; %% The frame where to place the ground

rootLink = 'root_link'; %% The base link

joints_positions=zeros(length(jointOrder),1); %% The joints'position

printModel = true; %% Define if the model has to be printed on the command window

%% No need to edit from here

% Main variable of iDyntreeWrappers used for many things including updating
% robot position and getting world to frame transforms
KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder, rootLink, modelPath,fileName,false);

if printModel
    KinDynModel.kinDynComp.model().toString() %%Print model
end

% Set initial position of the robot
iDynTreeWrappers.setRobotState(KinDynModel,eye(4),joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);

groundFrameTransform = iDynTreeWrappers.getWorldTransform(KinDynModel, groundFrame);

world_H_base = groundFrameTransform^-1;

iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);

% Prepare figure, handles and variables required for the update, some extra
% options are commented.
[visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix,...
    'color',[1,1,1],'transparency',1, 'name', ['Plot frame ', robotName], 'reuseFigure', 'name');
xlim([-0.5, 0.5])
ylim([-0.5, 0.5])
zlim([-0.05, 1.4])

for t = 1 : length(testFrames)
    frameTransform = iDynTreeWrappers.getWorldTransform(KinDynModel, testFrames{t});
    testFramesObjects{t} = iDynTreeWrappers.plotFrame(frameTransform, 0.2, 5);
end
