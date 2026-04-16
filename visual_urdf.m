%% 机械臂 URDF 模型渲染程序
clear; clc; close all;

% 1. 指定文件名
urdfFile = 'ur10_shovel.urdf';

% 2. 导入机械臂模型
% importrobot 会自动解析 URDF 及其关联的 Mesh 文件
try
    robot = importrobot(urdfFile);
    robot.DataFormat = 'column'; % 设置数据格式为列向量，方便运动学计算
    fprintf('成功加载模型: %s\n', urdfFile);
catch ME
    error('模型加载失败，请检查 URDF 文件及 meshes 路径是否正确。错误信息: %s', ME.message);
end

% 3. 获取初始位姿
q0 = homeConfiguration(robot);

% 4. 创建可视化窗口
figure('Name', '机械臂 CAD 模型可视化', 'Color', 'w');

% 5. 显示模型
% 'Collisions', 'off' -> 隐藏绿色/几何包围体
% 'Visuals', 'on'    -> 显示原始 CAD 网格模型
show(robot, q0, 'Collisions', 'off', 'Visuals', 'on');

% 6. 视图优化
view([135 30]); % 设置一个标准的等轴侧视图
grid on;        % 显示网格
axis equal;     % 保持坐标轴比例一致
title(['URDF 视觉模型渲染: ', urdfFile]);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 打印 Body 列表供参考
disp('模型包含以下连杆 (Links):');
disp(robot.BodyNames);