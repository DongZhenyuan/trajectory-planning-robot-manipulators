%% 启动脚本
% 本脚本将配置 MATLAB 搜索路径，使目录子文件夹中的所有程序都彼此可见
clear, clc, close all;
disp('Starting Robot Manipulator Trajectory Projects...')

rootDir = fileparts(which(mfilename));
addpath(genpath('matlab'),genpath('simulink'),genpath('utilities'));

if ~isfolder('myWork')
    mkdir('myWork');
end
% 该语句为“仿真缓存文件夹”和“代码生成文件夹”指定根文件夹
Simulink.fileGenControl('set','CacheFolder','myWork','CodeGenFolder','myWork');
disp('Ready to Go')

%%
% Trajectory Planning of Robot Manipulators with MATLAB and Simulink
% Startup script
%
% Copyright 2019 The Mathworks, Inc.

clear, clc, close all;
disp('Starting Robot Manipulator Trajectory Example...')

% Set up the MATLAB search path
rootDir = fileparts(which(mfilename));
addpath(genpath('matlab'),genpath('simulink'),genpath('utilities'));

% Set the code generation and cache folders to a work folder
% If the folder does not exist, create it
if ~isfolder('work')
    mkdir('work');
end
Simulink.fileGenControl('set','CacheFolder','work','CodeGenFolder','work');

%%
% 使用 MATLAB/Simulink 进行机械臂的轨迹规划
% 本脚本将配置 MATLAB 搜索路径，使目录子文件夹中的所有程序都彼此可见，以便所有文件都能正常运行。
% 启动脚本
clear, clc, close all;
disp('Starting Robot Manipulator Trajectory Example...')


% 配置 MATLAB 搜索路径
% rootDir = fileparts(which(mfilename));
% mfilename
% 返回一个字符向量，其中包含发生函数调用的文件的名称
% 从文件中调用时，则会返回该文件的名称。这样，脚本或函数就可以确定其名称。务必注意，从命令行调用时，mfilename 返回空字符向量
% which
% which item 显示 item 的完整路径
% fileparts()
% [filepath,name,ext] = fileparts(filename) 返回指定文件的路径名称、文件名和扩展名
% 该语句的弊端在于必须整体运行脚本，不支持分步调试。不妨一言以蔽之：
rootDir = pwd;
addpath(genpath('代码评估'),genpath('simulink仿真'),genpath('utilities拓展'));
% addpath()
% addpath(folderName1,...,folderNameN) 将指定的文件夹添加到当前 MATLAB® 会话的搜索路径的顶层
% addpath()函数默认检测文件夹更改，一旦运行函数，文件夹名称不可更改


% 将代码生成文件夹和缓存文件夹设置为工作文件夹，如果文件夹不存在则新建
if ~isfolder('myWork')
    mkdir('myWork');
end
% Simulink.fileGenControl为图更新和模型编译生成的文件指定根文件夹
% CacheFolder（仿真缓存文件夹）：指定用于仿真的模型编译工件（包括 Simulink® 缓存文件）的根文件夹
% CodeGenFolder（代码生成文件夹）：指定代码生成文件的根文件夹

Simulink.fileGenControl('set','CacheFolder','myWork','CodeGenFolder','myWork');

% 官方文档
% genpath()
% https://ww2.mathworks.cn/help/releases/R2021a/matlab/ref/genpath.html?requestedDomain=cn
% Simulink.fileGenControl
% https://ww2.mathworks.cn/help/rtw/ref/simulink.filegencontrol.html