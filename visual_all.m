%% 机械臂与障碍物场景统一可视化及尺寸校验
clear; clc; close all;

% --- 1. 加载机械臂 URDF ---
urdfFile = 'ur10_shovel.urdf';
try
    robot = importrobot(urdfFile);
    robot.DataFormat = 'column';
    q0 = homeConfiguration(robot);
    fprintf('成功加载机器人模型: %s\n', urdfFile);
catch ME
    error('URDF加载失败: %s', ME.message);
end

% --- 2. 处理 STL 障碍物环境 ---
stlFile = '避障场景.STL';
if exist(stlFile, 'file')
    TR = stlread(stlFile);
    % 按照您的逻辑进行 Y-Z 轴交换
    points_swapped = TR.Points;
    points_swapped(:, [2, 3]) = points_swapped(:, [3, 2]); 
else
    error('未找到 STL 文件: %s', stlFile);
end

% --- 3. 尺寸计算与对比 ---
% A. 计算 STL 尺寸
stl_min = min(points_swapped);
stl_max = max(points_swapped);
stl_dims = stl_max - stl_min;

% B. 计算机器人 (URDF) 的大致尺寸 (基于各关节坐标系原点)
numBodies = numel(robot.BodyNames);
joint_positions = zeros(numBodies, 3);
for i = 1:numBodies
    tform = getTransform(robot, q0, robot.BodyNames{i});
    joint_positions(i, :) = tform(1:3, 4)';
end
robot_min = min(joint_positions);
robot_max = max(joint_positions);
robot_dims = robot_max - robot_min;

fprintf('\n%s\n', repmat('=',1,30));
fprintf('       模型尺寸校验报告\n');
fprintf('%s\n', repmat('=',1,30));
fprintf('障碍物(STL) 尺寸: X:%.2f, Y:%.2f, Z:%.2f\n', stl_dims(1), stl_dims(2), stl_dims(3));
fprintf('机械臂(URDF) 尺寸: X:%.2f, Y:%.2f, Z:%.2f (骨架范围)\n', robot_dims(1), robot_dims(2), robot_dims(3));
fprintf('%s\n', repmat('-',1,30));

% --- 4. 单位冲突检测与自动缩放 ---
scale_factor = 1; % 默认不缩放
if any(stl_dims > 100) && any(robot_dims < 2)
    fprintf('检测到潜在单位冲突：STL 尺寸较大(可能是mm)，而 URDF 尺寸较小(可能是m)。\n');
    
    % 询问用户是否自动缩放
    user_input = input('是否将 STL 缩小 1000 倍以匹配 URDF 单位？(y/n): ', 's');
    if strcmpi(user_input, 'y') || strcmpi(user_input, 'yes')
        scale_factor = 1/1000;
        points_swapped = points_swapped * scale_factor;
        
        % 重新计算缩放后的尺寸
        stl_min = min(points_swapped);
        stl_max = max(points_swapped);
        stl_dims = stl_max - stl_min;
        
        fprintf('STL 已缩小 1000 倍。新尺寸: X:%.2f, Y:%.2f, Z:%.2f\n', ...
                stl_dims(1), stl_dims(2), stl_dims(3));
    else
        fprintf('保持原始 STL 尺寸。\n');
    end
end

% --- 5. 统一窗口渲染 ---
fig = figure('Name', '机器人-环境集成仿真', 'Color', 'w');
ax = axes('Parent', fig);
hold on;

% 修正后的 show 函数调用 (不带输出参数)
show(robot, q0, 'Parent', ax, 'Visuals', 'on', 'Collisions', 'off');

% 绘制障碍物
h_stl = trisurf(TR.ConnectivityList, points_swapped(:,1), points_swapped(:,2), points_swapped(:,3), ...
    'Parent', ax, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% --- 6. 视图优化 ---
view(135, 30);
grid on; axis equal;
camlight; lighting gouraud;
title('机械臂与避障环境集成预览');
xlabel('X'); ylabel('Y'); zlabel('Z');
legend(h_stl, '环境障碍物', 'Location', 'northeast');

% --- 7. 添加比例尺参考 ---
% 在角落添加一个1米长的参考线
ref_line_start = [robot_min(1), robot_min(2), robot_min(3)];
ref_line_end = ref_line_start + [1, 0, 0]; % 1米长的X方向参考线
plot3([ref_line_start(1), ref_line_end(1)], ...
      [ref_line_start(2), ref_line_end(2)], ...
      [ref_line_start(3), ref_line_end(3)], ...
      'r-', 'LineWidth', 2);
text(ref_line_end(1), ref_line_end(2), ref_line_end(3), '1m', ...
     'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);

fprintf('\n可视化完成。\n');