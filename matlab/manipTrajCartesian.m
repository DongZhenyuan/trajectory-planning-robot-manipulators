% 机械臂轨迹生成
% 只生成笛卡尔坐标系下的轨迹(没有旋转)

%% 启动
clear, clc, close all

% 定义航点信息
createWaypointData;

% 定义 IK
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = gen3.homeConfiguration;

% 启动绘图
plotMode = 1; % 0 = None, 1 = 轨迹, 2 = 坐标框架
show(gen3,jointAnglesHome','Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% 生成轨迹
% 仅笛卡尔运动
trajType = 'bspline';
switch trajType
    case 'trap'
        [q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[3 1]));
                            
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels);
        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels, ...
            'AccelerationBoundaryCondition',waypointAccels);
        
    case 'bspline'
        ctrlpoints = waypoints; % 可根据需要进行调整
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% 用刚体树显示完整的轨迹
if plotMode == 1
    set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));
elseif plotMode == 2
    plotTransforms(q',repmat([1 0 0 0],[size(q,2) 1]),'FrameSize',0.05);
end

% 为了使轨迹可视化，运行以下这条语句
plotTrajectory(trajTimes,q,qd,qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)

%% 轨迹跟踪回路
for idx = 1:numel(trajTimes) 
    % 求解 IK
    tgtPose = trvec2tform(q(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;

    % 展示机械臂
    show(gen3,config,'Frames','off','PreservePlot',false);
    title(['时间为 ' num2str(trajTimes(idx)) ' 秒时的轨迹位置'])
    drawnow    
end