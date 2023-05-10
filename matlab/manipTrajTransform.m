% 机械臂轨迹生成
% 使用线性插值生成组合变换(旋转和平移)轨迹

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

%% 生成并跟踪轨迹
% 一次循环一个片段
trajType = 'trap';
numWaypoints = size(waypoints,2);
for w = 1:numWaypoints-1
        
    % 为每个片段获得段的初始和最终变换及其次数
    T0 = trvec2tform(waypoints(:,w)') * eul2tform(orientations(:,w)');
    Tf = trvec2tform(waypoints(:,w+1)') * eul2tform(orientations(:,w+1)');
    timeInterval = waypointTimes(w:w+1);
    trajTimes = timeInterval(1):ts:timeInterval(2);
    
    % 求轨迹生成的变换
    [T,vel,acc] = transformtraj(T0,Tf,timeInterval,trajTimes);  
    
    % 每段轨迹的可视化
    if plotMode == 1
        eePos = tform2trvec(T);
        set(hTraj,'xdata',eePos(:,1),'ydata',eePos(:,2),'zdata',eePos(:,3));
    elseif plotMode == 2
        plotTransforms(tform2trvec(T),tform2quat(T),'FrameSize',0.05);
    end
    for idx = 1:numel(trajTimes) 
        % 求解 IK
        tgtPose = T(:,:,idx);
        [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
        ikInitGuess = config;

        % 展示机械臂
        show(gen3,config,'Frames','off','PreservePlot',false);
        title(['时间为 ' num2str(trajTimes(idx)) ' 秒时的轨迹位置'])
        drawnow
    end
    
end