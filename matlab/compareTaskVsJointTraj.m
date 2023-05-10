% 比较关节空间和任务空间中的轨迹

%% 启动
clc
createWaypointData;
figure, hold on
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko:','LineWidth',2);
title('轨迹路线点'); 
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on
view([45 45]);
% 定义 IK 求解器
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
% 在此使用较小的样本时间，再加上在任务空间轨迹中评估IK，所以关节空间和任务空间之间的区别很明显。
ts = 0.02;
trajTimes = 0:ts:waypointTimes(end);
% 为图形初始化矩阵
qTask = zeros(numJoints,numel(trajTimes)); % 导出任务空间轨迹中的关节角
posJoint = zeros(3,numel(trajTimes));      % 导出关节空间轨迹中的末端执行器位置

%% 创建和评估任务空间轨迹
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

disp('运行任务空间轨迹，生成评估中...')
tic;

% 轨迹生成
[posTask,velTask,accelTask] = trapveltraj(waypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[3 1]), ...
    'EndTime',repmat(diff(waypointTimes),[3 1]));

% 轨迹评估
for idx = 1:numel(trajTimes) 
    % 求解 IK
    tgtPose = trvec2tform(posTask(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    qTask(:,idx) = config;
end

taskTime = toc;
disp(['任务空间轨迹时间：' num2str(taskTime) ' 秒']);

%% 创建和评估关节空间轨迹
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

disp('运行任务空间轨迹，生成评估中......')
tic;

% 为所有航点求解 IK
numWaypoints = size(waypoints,2);
numJoints = numel(gen3.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);
for idx = 1:numWaypoints
    tgtPose = trvec2tform(waypoints(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    cfgDiff = config - ikInitGuess;
    jointWaypoints(:,idx) = config';    
end

% 轨迹生成
[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));

% 轨迹评估 (只需要找到末端执行器的位置)
for idx = 1:numel(trajTimes)  
    eeTform = getTransform(gen3,qJoint(:,idx)',eeName); 
    posJoint(:,idx) = tform2trvec(eeTform)'; 
end
jointTime = toc;
disp(['关节空间轨迹时间：' num2str(jointTime) ' 秒']);

%% 创建比较图像
% 比较笛卡尔空间中的轨迹
close all
figure, hold on
plot3(posTask(1,:),posTask(2,:),posTask(3,:),'b-');
plot3(posJoint(1,:),posJoint(2,:),posJoint(3,:),'r--');
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko','LineWidth',2);
title('轨迹比较'); 
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('任务空间轨迹','关节空间轨迹','航点');
grid on
view([45 45]);

% 比较关节角
% 绘制每个关节的轨迹
for idx = 1:numJoints
    figure, hold on;
    plot(trajTimes,qTask(idx,:),'b-');
    plot(trajTimes,qJoint(idx,:),'r-');
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['关节 ' num2str(idx) ' 的轨迹']); 
    xlabel('时间 [秒]');
    ylabel('关节角 [弧度]');
    legend('任务空间轨迹','关节空间轨迹');
end