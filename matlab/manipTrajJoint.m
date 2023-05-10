% 机械臂轨迹生成
% 通过在每个航路点上执行逆运动学，并在关节角度之间进行插值来生成关节空间轨迹

%% Setup
clear, clc, close all

% 定义航点信息
createWaypointData;

% 定义 IK
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

% 启用绘图
plotMode = 1; % 0 = None, 1 = 轨迹, 2 = 坐标框架
show(gen3,gen3.homeConfiguration,'Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% 在所有航点上求解逆运动学
includeOrientation = false; % 将此设置为使用零，或者非零方向

numWaypoints = size(waypoints,2);
numJoints = numel(gen3.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);

for idx = 1:numWaypoints
    if includeOrientation
        tgtPose = trvec2tform(waypoints(:,idx)') * eul2tform(orientations(:,idx)');
    else
        tgtPose =  trvec2tform(waypoints(:,idx)');
    end
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    jointWaypoints(:,idx) = config';
end

%% 在关节空间上生成轨迹
trajType = 'quintic';
switch trajType
    case 'trap'
        [q,qd,qdd] = trapveltraj(jointWaypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
                            
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(numJoints,numWaypoints));
        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(numJoints,numWaypoints), ...
            'AccelerationBoundaryCondition',zeros(numJoints,numWaypoints));
        
    case 'bspline'
        ctrlpoints = jointWaypoints; % 可根据需要进行调整
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% 为了使轨迹可视化，运行下面这条语句
% plotTrajectory(trajTimes,q,qd,qdd,'Names',"Joint " + string(1:numJoints),'WaypointTimes',waypointTimes)

%% 轨迹跟踪回路
for idx = 1:numel(trajTimes)  

    config = q(:,idx)';
    
    % 找到笛卡尔点进行可视化
    eeTform = getTransform(gen3,config,eeName);
    if plotMode == 1
        eePos = tform2trvec(eeTform);
        set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                  'ydata',[hTraj.YData eePos(2)], ...
                  'zdata',[hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.05);
    end

    % 展示机械臂
    show(gen3,config,'Frames','off','PreservePlot',false);
    title(['时间为 ' num2str(trajTimes(idx)) ' 秒时的轨迹位置'])
    drawnow   
    
end