% 机械臂轨迹生成
% 生成具有独立线性插值旋转轨迹的笛卡尔轨迹

%% 启动
clear, clc, close all

% 定义航点信息
createWaypointData;

% 定义 IK
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = gen3.homeConfiguration;

% 启用绘图
plotMode = 2; % 0 = None, 1 = 轨迹, 2 = 坐标框架
show(gen3,gen3.homeConfiguration,'Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% 生成并跟随轨迹
% 每次循环中，在一个单位时间内执行一个片段
trajType = 'trap';
numWaypoints = size(waypoints,2);
for w = 1:numWaypoints-1
    % 为每个片段获取初始和最终变换以及次数
    R0 = eul2quat(orientations(:,w)');
    Rf = eul2quat(orientations(:,w+1)');
    timeInterval = waypointTimes(w:w+1);
    trajTimes = timeInterval(1):ts:timeInterval(2);

    % 仅笛卡尔运动
    switch trajType
        case 'trap'
            [q,qd,qdd] = trapveltraj(waypoints(:,w:w+1),numel(trajTimes), ...
                'AccelTime',waypointAccelTimes(w), ... 
                'EndTime',diff(waypointTimes(w:w+1)));

        case 'cubic'
            [q,qd,qdd] = cubicpolytraj(waypoints(:,w:w+1),waypointTimes(w:w+1),trajTimes, ... 
                'VelocityBoundaryCondition',waypointVels(:,w:w+1));

        case 'quintic'
            [q,qd,qdd] = quinticpolytraj(waypoints(:,w:w+1),waypointTimes(w:w+1),trajTimes, ... 
                'VelocityBoundaryCondition',waypointVels(:,w:w+1), ...
                'AccelerationBoundaryCondition',waypointAccels(:,w:w+1));

        case 'bspline'
            ctrlpoints = waypoints(:,idx:idx+1); % 可根据需要进行调整
            [q,qd,qdd] = bsplinepolytraj(ctrlpoints,timeInterval,trajTimes);

        otherwise
            error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
    end
        
    % 从轨迹生成中找到四元数
    [R, omega, alpha] = rottraj(R0, Rf, timeInterval, trajTimes);    
    
    % 绘制轨迹
    if plotMode == 1
        set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));
    elseif plotMode == 2
        plotTransforms(q',R','FrameSize',0.05)
    end
    
    % 轨迹跟踪回路
    for idx = 1:numel(trajTimes) 
        % 求解 IK
        tgtPose = trvec2tform(q(:,idx)') * quat2tform(R(:,idx)');
        [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
        ikInitGuess = config;

        % 展示机械臂
        show(gen3,config,'Frames','off','PreservePlot',false);
        title(['时间为 ' num2str(trajTimes(idx)) ' 秒时的轨迹位置'])
        drawnow    
    end
    
    
end