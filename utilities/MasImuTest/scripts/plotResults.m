icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');

meshFilePrefix = [icubModelsInstallPrefix '/share'];

robotName='iCubGenova04';

modelPath = [icubModelsInstallPrefix '/share/iCub/robots/' robotName '/'];
fileName='model.urdf';

datasets = {left_data, right_data};
options = {left_options, right_options};



for dataIndex = 1 : length(datasets)
    data = datasets{dataIndex};
    opt = options{dataIndex};
    rpyImu = zeros(3, length(datasets));
    rpyFK = zeros(3, length(datasets),1);
    
    jointOrder = opt.ConsideredJoints;
    world_H_base = eye(4);
    world_H_base(1:3, 1:3) = settings.base_rotation;
    
    KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,settings.base_link,modelPath,fileName,false);
    joints_positions = data(1).JointPositions_rad;
    iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);
    figure('WindowState', 'maximized')
    [visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix, 'transparency',1, 'reuseFigure', 'gcf');
    xlim([-0.8, 0.8])
    ylim([-0.8, 0.8])
    zlim([-1, 0.8])
    frameTransform = iDynTreeWrappers.getWorldTransform(KinDynModel, opt.FrameName);
    wRimu = frameTransform(1:3, 1:3);
    rpyFK(:, 1) = wbc.rollPitchYawFromRotation(frameTransform(1:3, 1:3));
    forward_kin_frame = iDynTreeWrappers.plotFrame(frameTransform, 0.2, 10);
    rpy(1) = data(1).RPYfromIMUinDeg(3) * pi/180;
    rpy(2) = -data(1).RPYfromIMUinDeg(2) * pi/180;    
    rpy(3) = -data(1).RPYfromIMUinDeg(1) * pi/180;
    rpyImu(:, 1) = rpy;
    frameTransform(1:3, 1:3) = wbc.rotz(rpy(3))*wbc.roty(rpy(2))*wbc.rotx(rpy(1));
    wRimu = wRimu * frameTransform(1:3, 1:3)';
    frameTransform(1:3, 1:3) = wRimu * frameTransform(1:3, 1:3);
    imu_frame = iDynTreeWrappers.plotFrame(frameTransform, 0.4, 5);
    or = frameTransform(1:3, 4);
    acc = frameTransform * [data(1).Accelerometer'/9.81/2; 1]; 
    gravity = plot3([or(1) acc(1)], [or(2) acc(2)], [or(3) acc(3)], 'm', 'linewidth', 7);
    pause

    for i = 2 : length(data)
        joints_positions = data(i).JointPositions_rad;
        iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);
        frameTransform = iDynTreeWrappers.getWorldTransform(KinDynModel, opt.FrameName);
        rpyFK(:, i) = wbc.rollPitchYawFromRotation(frameTransform(1:3, 1:3));
        iDynTreeWrappers.updateFrame(forward_kin_frame, frameTransform);
        rpy(1) = data(i).RPYfromIMUinDeg(3) * pi/180;
        rpy(2) = -data(i).RPYfromIMUinDeg(2) * pi/180;
        rpy(3) = -data(i).RPYfromIMUinDeg(1) * pi/180;
        rpyImu(:, i) = rpy;
        frameTransform(1:3, 1:3) = wRimu * wbc.rotz(rpy(3))*wbc.roty(rpy(2))*wbc.rotx(rpy(1));
        iDynTreeWrappers.updateFrame(imu_frame, frameTransform);
        iDynTreeWrappers.updateVisualization(KinDynModel,visualizer);
        or = frameTransform(1:3, 4);
        acc = frameTransform * [data(i).Accelerometer'/9.81/2; 1];
        set(gravity, 'XData', [or(1) acc(1)], 'YData', [or(2) acc(2)], 'ZData', [or(3) acc(3)]);

        drawnow()
        pause(0.01)
    end
    figure
    plot(rpyFK')
    title('FK')
    legend('Roll', 'Pitch', 'Yaw')
    figure
    plot(rpyImu')
    title('IMU')
    legend('Roll', 'Pitch', 'Yaw')

    if (dataIndex ~= length(datasets))
        pause
    end
    
end