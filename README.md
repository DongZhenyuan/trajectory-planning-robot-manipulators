## 使用 MATLAB 和 Simulink 对机械臂进行轨迹规划

该项目来自：
https://github.com/mathworks-robotics/trajectory-planning-robot-manipulators

Copyright 2019 The MathWorks, Inc.

## Description

此项目包括 MATLAB 和 Simulink 机器人机械手的轨迹生成与评估

所有文件均采用 7 自由度 Kinova Gen3 超轻型机器人机械臂：
https://www.kinovarobotics.com/en/products/robotic-arms/gen3-ultra-lightweight-robot

本项目提供了预先保存的 Kinova Gen3 的MATLAB刚体树模型，当然也可以 从Kinova Kortex GitHub存储库访问3D模型描述：
https://github.com/Kinovarobotics/ros_kortex

有关机械臂机器人系统工具箱功能的更多信息，请参阅文档：
see the documentation: https://www.mathworks.com/help/robotics/manipulators.html

有关轨迹规划的更多背景信息，请参阅此演示文稿：
https://cw.fel.cvut.cz/old/_media/courses/a3m33iro/080manipulatortrajectoryplanning.pdf

## 文件
若要开始，请运行 `startupExample.m` 脚本。这将配置 MATLAB 搜索路径，以便所有示例都能正常运行。

### `matlab` 文件夹
包含用于轨迹规划的 MATLAB 文件

* `manipTrajCartesian.m`: 任务空间（仅平移）轨迹
* `manipTrajJoint.m`: 关节空间轨迹。包含一个 `includeOrientation` 变量用于打开或关闭路径点方向
* `manipTrajLinearRotation.m`: 具有线性插值方向的任务空间（仅平移）轨迹
* `manipTrajTransform.m`: 线性插值变换轨迹（平移和定向）
* `manipTrajTransformTimeScaling.m`: 使用非线性时间缩放进行插值的变换轨迹（平移和定向）
* `compareTaskVsJointTraj.m`: 说明任务空间和关节空间轨迹之间区别的比较脚本

**注意：** 上述所有脚本都是可配置的：
* `createWaypointData.m` 脚本: 生成示例航点、轨迹时间和其他必要的规划变量。
* `trajType` 变量: 用于切换轨迹类型
* `plotMode` 变量: 用于切换航点/轨迹可视化类型

### `simulink` 文件夹
包含用于轨迹规划的 Simulink 文件。

* `manipCartesianTrajectory.slx`: 任务空间（仅平移）轨迹
* `manipJointTrajectory.slx`: 关节空间轨迹
* `manipRotationTrajectory.slx`: 具有线性插值方向的任务空间（仅平移）轨迹
* `manipTransformTrajectory.slx`: 线性插值变换轨迹（平移和定向）
* `manipTransformTrajectoryTimeScaling.slx`: 使用非线性时间缩放进行插值的变换轨迹（平移和定向）

**注意：** 还有一些型号适用于机器人操作系统 （ROS），其名称与前缀相同。 航点信息不是在 MATLAB 基本工作区中使用变量，而是使用 ROS 消息进行通信。 
要对此进行测试，，您可以使用航点发布者应用程序或 `publishWaypoints` 脚本（请参阅下一节）。

ROS主题和消息类型包括：
* `/waypoints`: 航点列表，消息类型 `geometry_msgs/PoseArray`
* `/waypoint_times`: 航点列表，消息类型 `std_msgs/Float64MultiArray`

### `utilities` 文件夹
包含上述 MATLAB 和 Simulink 文件的几个实用程序

* `createWaypointData.m`: 创建采样路径点、路径点时间和其他必要的规划变量。如果要更改路径点或其他轨迹参考值，请自行修改
* `cylinder.stl`: "Dummy" 网格文件，代表末端执行器连接到手臂
* `gen3.mat`: 预先保存的刚体树，包含机器人手臂的3D模型
* `gen3positions.mat`: 预先保存的关节和末端执行器配置，用于机器人手臂的“返航”和“缩回”位置
* `importGen3Model.m`: 此示例用于导入 Kinova Gen3 机械手模型的功能，在此不需要。如果要自己从源 URDF 文件导入新模型，则可以使用它。
* `plotTrajectory.m`: 用于绘制生成的轨迹剖面的实用功能（与 MATLAB 文件一起使用）
* `publishWaypoints.m`: 测试将航点信息发布为 ROS 消息
* `trajExampleUtils.slx`: 包含 Simulink 示例的通用组件的块库
* `visualizeRobot.m`: 上述库用于从 Simulink 模型绘制机械臂的实用程序函数
* `waypointPublisher.mlapp`: MATLAB 应用程序用于修改路径点并将其发布到基本工作区或作为 ROS 消息
