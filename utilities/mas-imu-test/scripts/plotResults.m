%% Settings

icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');

meshFilePrefix = [icubModelsInstallPrefix '/share'];

robotName='iCubGenova04';

modelPath = [icubModelsInstallPrefix '/share/iCub/robots/' robotName '/'];
fileName='model.urdf';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% No need to edit from here on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Iterate over the tests
tests_fields = fieldnames(tests);

for testIndex = 1 : numel(tests_fields)
    %% Buffers initialization
    data = tests.(tests_fields{testIndex}).data;
    opt = tests.(tests_fields{testIndex}).options;
    rpyImu = zeros(3, size(data, 1));
    rpyFK = zeros(3, size(data, 1));
    
    %% Base initialization
    jointOrder = opt.ConsideredJoints;
    world_H_base = eye(4);
    world_H_base(1:3, 1:3) = settings.base_rotation;
    
    %% Model loading and set robot state
    KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,settings.base_link,modelPath,fileName,false);
    joints_positions = data(1).JointPositions_rad;
    iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);
    
    %% Visualization setup
    [visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix, 'transparency',1, 'name', 'Imu Test', 'reuseFigure', 'name');
    set(gcf, 'WindowState', 'maximized');
    xlim([-0.8, 0.8])
    ylim([-0.8, 0.8])
    zlim([-1, 0.8])
    title(opt.TestName);
    
    %% Frame from forward kinematics and plotting
    frameTransform = iDynTreeWrappers.getWorldTransform(KinDynModel, opt.FrameName);
    frameTransform(1:3, 1:3) = data(1).RotationFromEncoders;
    rpyFK(:, 1) = wbc.rollPitchYawFromRotation(frameTransform(1:3, 1:3));
    forward_kin_frame = iDynTreeWrappers.plotFrame(frameTransform, 0.2, 10);
    
    %% Frame from IMU
    rpyImu(:, 1) = data(1).RPYfromIMUinDegRemapped;
    frameTransform(1:3, 1:3) = data(1).RotationFromIMUInInertialYawFiltered;
    imu_frame = iDynTreeWrappers.plotFrame(frameTransform, 0.4, 5);
    
    %% Plot of accelerometer
    or = frameTransform(1:3, 4);
    acc = frameTransform * [data(1).Accelerometer'/9.81/2; 1]; 
    gravity = plot3([or(1) acc(1)], [or(2) acc(2)], [or(3) acc(3)], 'm', 'linewidth', 7);
    
    %% Data loop
    for i = 2 : length(data)
        %% Update robot state
        joints_positions = data(i).JointPositions_rad;
        iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);
        
        %% Update frame from forward kinematics
        frameTransform = iDynTreeWrappers.getWorldTransform(KinDynModel, opt.FrameName);
        frameTransform(1:3, 1:3) = data(i).RotationFromEncoders;
        rpyFK(:, i) = wbc.rollPitchYawFromRotation(frameTransform(1:3, 1:3));
        iDynTreeWrappers.updateFrame(forward_kin_frame, frameTransform);
        
        %% Update frame from IMU
        rpyImu(:, i) = data(i).RPYfromIMUinDegRemapped;
        frameTransform(1:3, 1:3) = data(i).RotationFromIMUInInertialYawFiltered;
        iDynTreeWrappers.updateFrame(imu_frame, frameTransform);
        
        %% Update robot visualization
        iDynTreeWrappers.updateVisualization(KinDynModel,visualizer);
        
        %% Update gravity visualization
        or = frameTransform(1:3, 4);
        acc = frameTransform * [data(i).Accelerometer'/9.81/2; 1];
        set(gravity, 'XData', [or(1) acc(1)], 'YData', [or(2) acc(2)], 'ZData', [or(3) acc(3)]);

        %% Force rendering
        drawnow()
        pause(0.01)
    end
    figure
    plot(rpyFK')
    title(strcat(opt.TestName, " RPY from Forward Kinematics"))
    legend('Roll', 'Pitch', 'Yaw')
    figure
    plot(rpyImu')
    title(strcat(opt.TestName, " RPY from IMU"))
    legend('Roll', 'Pitch', 'Yaw')

    if (testIndex ~= numel(tests_fields))
        pause
    end
    
end