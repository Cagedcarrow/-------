%% =========================================================================
% 论文项目：基于改进RRT算法的抹泥机械臂路径规划系统
% 主执行程序 (Main Entry Point)
% 功能：整合 GUI 交互、运动学解算、障碍物环境与自碰撞检测的自动化控制系统
% =========================================================================
clc; clear; close all;

% 1. 环境配置：添加模块化文件夹路径
% 确保主程序能访问各子文件夹中的函数
addpath('gui', 'kinematics', 'planner');

% 2. 运动学系统初始化
% 加载机器人模型并预配置逆运动学(IK)求解器
[robot, ik, ~] = initRobot(); 
endEffector = 'shovel_tip'; % 定义末端执行器参考点

% 3. 加载并处理障碍物 STL 环境
stlFile = '避障场景.STL';
if exist(stlFile, 'file')
    TR = stlread(stlFile);
    % 按照 visual_all.m 的逻辑进行 Y-Z 轴交换
    points_swapped = TR.Points;
    points_swapped(:, [2, 3]) = points_swapped(:, [3, 2]); 
    
    % 尺寸计算与单位检测
    stl_min = min(points_swapped);
    stl_max = max(points_swapped);
    stl_dims = stl_max - stl_min;
    
    % 计算机器人尺寸 (基于各关节坐标系原点)
    q0 = homeConfiguration(robot);
    numBodies = numel(robot.BodyNames);
    joint_positions = zeros(numBodies, 3);
    for i = 1:numBodies
        tform = getTransform(robot, q0, robot.BodyNames{i});
        joint_positions(i, :) = tform(1:3, 4)';
    end
    robot_min = min(joint_positions);
    robot_max = max(joint_positions);
    robot_dims = robot_max - robot_min;
    
    % 单位冲突检测与自动缩放
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
    
    fprintf('\n%s\n', repmat('=',1,30));
    fprintf('       模型尺寸校验报告\n');
    fprintf('%s\n', repmat('=',1,30));
    fprintf('障碍物(STL) 尺寸: X:%.2f, Y:%.2f, Z:%.2f\n', stl_dims(1), stl_dims(2), stl_dims(3));
    fprintf('机械臂(URDF) 尺寸: X:%.2f, Y:%.2f, Z:%.2f (骨架范围)\n', robot_dims(1), robot_dims(2), robot_dims(3));
    fprintf('%s\n', repmat('-',1,30));
else
    error('未找到障碍物 STL 文件: %s', stlFile);
end

% 4. 启动图形化界面 (GUI)
% 绑定"开始运行"按钮的回调函数为本文件中的 startSimulation
handles = MoveShovelTip(robot, @(src, event) startSimulation());

% 5. 全局数据持久化
% 使用 guidata 存储机器人对象及当前状态，确保回调函数可访问
handles.robot = robot;
handles.ik = ik;
handles.endEffector = endEffector;
handles.currentConfig = homeConfiguration(robot); % 初始姿态

% 存储障碍物数据
handles.obstacle.TR = TR;
handles.obstacle.points = points_swapped;
handles.obstacle.scale_factor = scale_factor;

% 在 GUI 中绘制障碍物
h_stl = trisurf(TR.ConnectivityList, points_swapped(:,1), points_swapped(:,2), points_swapped(:,3), ...
    'Parent', handles.ax, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
handles.obstacle.h_stl = h_stl;

% 添加障碍物图例
legend(h_stl, '环境障碍物', 'Location', 'northeast');

% 添加比例尺参考线
ref_line_start = [robot_min(1), robot_min(2), robot_min(3)];
ref_line_end = ref_line_start + [1, 0, 0]; % 1米长的X方向参考线
plot3(handles.ax, [ref_line_start(1), ref_line_end(1)], ...
      [ref_line_start(2), ref_line_end(2)], ...
      [ref_line_start(3), ref_line_end(3)], ...
      'r-', 'LineWidth', 2);
text(handles.ax, ref_line_end(1), ref_line_end(2), ref_line_end(3), '1m', ...
     'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);

% 优化视图
view(handles.ax, 135, 30);
grid(handles.ax, 'on');
axis(handles.ax, 'equal');
camlight(handles.ax);
lighting(handles.ax, 'gouraud');
title(handles.ax, '机械臂与避障环境集成控制系统');

% --- 新增：启动基座控制窗口 ---
% 注意：我们将主窗口句柄作为参数传给回调，防止找不到路
mainFigHandle = handles.fig;
baseH = BaseControl(@(src, event) updateBasePose(mainFigHandle));
% 将基座句柄存入 handles
handles.baseH = baseH;
guidata(handles.fig, handles);

fprintf('✅ 系统已就绪。请在左侧设置起点/终点，点击"开始规划运行"。\n');

%% =========================================================================
% 核心执行模块：路径规划与避障驱动
% =========================================================================
function startSimulation()
    % 获取最新的程序句柄
    figHandle = gcf;
    h = guidata(figHandle);
    
    % 1. 同步 GUI 数据并刷新 3D 标记
    [startPos, endPos] = refreshDisplay(h);
    if isempty(startPos), return; end % 防止初始化未完成时的误触
    
    % 2. 生成离散路径点 (目前采用线性插值作为基准)
    numSteps = 60;
    pathPoints = linearInterpolator(startPos, endPos, numSteps);
    
    % 3. 初始化轨迹显示与参数
    if isfield(h, 'hTrail') && ishandle(h.hTrail), delete(h.hTrail); end
    h.hTrail = plot3(h.ax, NaN, NaN, NaN, 'b-', 'LineWidth', 2);
    title(h.ax, '正在执行路径执行与碰撞监测...', 'Color', 'k');
    
    % 优化 IK 权重：优先保证位置准确，姿态适度跟随
    weights = [0.1 0.1 0.1 1 1 1]; 
    fixedOrientation = eul2tform([0, 0, 0], 'ZYX'); % 铲子初始姿态（水平）
    
    trailData = [];
    currentConfig = h.currentConfig;
    
    % 4. 路径执行循环
    for i = 1:numSteps
        targetXYZ = pathPoints(i, :);
        targetTform = trvec2tform(targetXYZ) * fixedOrientation;
        
        % A. 逆运动学求解
        [newConfig, ikSuccess] = solveIK(h.ik, h.endEffector, targetTform, weights, currentConfig);
        
        if ikSuccess
            % B. 自碰撞检测 (核心安全屏障)
            % 检查新解出的关节角度是否会导致铲子与机械臂本体干涉
            isColliding = collisionChecker(h.robot, newConfig);
            
            % C. 新增：环境障碍物碰撞检测
            % 检查末端执行器是否与障碍物碰撞
            if ~isColliding
                % 获取末端执行器当前位置
                actualTform = getTransform(h.robot, newConfig, h.endEffector);
                actualXYZ = tform2trvec(actualTform);
                
                % 简单的距离检测（可根据需要扩展为更复杂的碰撞检测）
                obstacle_points = h.obstacle.points;
                distances = sqrt(sum((obstacle_points - actualXYZ).^2, 2));
                min_distance = min(distances);
                
                if min_distance < 0.05 % 5cm安全距离
                    title(h.ax, '⚠️ 警告：末端接近障碍物！', 'Color', [0.8 0.5 0]);
                    plot3(h.ax, actualXYZ(1), actualXYZ(2), actualXYZ(3), 'yo', ...
                          'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'y');
                    % 继续执行但标记警告
                end
                
                if min_distance < 0.01 % 1cm碰撞阈值
                    title(h.ax, '❌ 碰撞警告：末端与障碍物发生干涉！', 'Color', 'r');
                    plot3(h.ax, actualXYZ(1), actualXYZ(2), actualXYZ(3), 'rx', ...
                          'MarkerSize', 15, 'LineWidth', 3);
                    break;
                end
            end
            
            if ~isColliding
                % D. 安全：更新关节配置并获取真实空间坐标用于绘图
                currentConfig = newConfig;
                actualTform = getTransform(h.robot, currentConfig, h.endEffector);
                actualXYZ = tform2trvec(actualTform);
                
                % E. 渲染机器人与运动轨迹
                show(h.robot, currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
                
                trailData = [trailData; actualXYZ]; %#ok<AGROW>
                set(h.hTrail, 'XData', trailData(:,1), 'YData', trailData(:,2), 'ZData', trailData(:,3));
                
                drawnow; % 确保动画实时性
            else
                % F. 自碰撞异常处理：停止运动并提示
                title(h.ax, '❌ 警告：路径中发生自碰撞干涉！已紧急制动', 'Color', 'r');
                plot3(h.ax, targetXYZ(1), targetXYZ(2), targetXYZ(3), 'rx', 'MarkerSize', 15, 'LineWidth', 3);
                break; 
            end
        else
            warning('路径点 %d 逆运动学无解。', i);
            break;
        end
    end
    
    % 5. 更新最终状态
    h.currentConfig = currentConfig;
    guidata(figHandle, h);
    if ~exist('isColliding','var') || ~isColliding
        title(h.ax, '✅ 路径运动任务完成');
    end
end

%% =========================================================================
% 基座控制回调函数
% =========================================================================
function updateBasePose(mainFig)
    % 1. 获取数据
    h = guidata(mainFig);
    hb = guidata(gcf); % 当前触发回调的基座控制窗口
    if isempty(h) || isempty(hb), return; end

    % 2. 提取滑块数值
    bx = hb.sliders{1}.Value;
    by = hb.sliders{2}.Value;
    bz = hb.sliders{3}.Value;
    br = deg2rad(hb.sliders{4}.Value);
    bp = deg2rad(hb.sliders{5}.Value);
    byaw = deg2rad(hb.sliders{6}.Value);
    
    % 更新数值文本显示
    for k=1:6, hb.txtVals{k}.String = sprintf('%.2f', hb.sliders{k}.Value); end

    % 3. 构造变换矩阵
    tformBase = trvec2tform([bx, by, bz]) * eul2tform([byaw, bp, br], 'ZYX');

    % --- 核心修复点 ---
    % 获取机器人模型中第一个真正的连杆（它连接在 Base 上）
    % 通常在导入后，Bodies{1} 就是第一个连杆
    rootBody = h.robot.Bodies{1}; 
    
    % 修改该连杆关节相对于基座的固定变换
    setFixedTransform(rootBody.Joint, tformBase);
    % ------------------

    % 4. 刷新显示
    show(h.robot, h.currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
    refreshDisplay(h); 
    drawnow limitrate;
end

%% =========================================================================
% 辅助函数：刷新显示
% =========================================================================
function [startPos, endPos] = refreshDisplay(h)
    % 此函数应从 GUI 获取起点和终点位置
    % 这里假设 GUI 已经更新了这些值
    % 实际实现可能需要从 GUI 控件读取
    
    % 临时返回默认值（需要根据实际 GUI 实现调整）
    startPos = [0.5, 0, 0.5];
    endPos = [0.5, 0.5, 0.5];
end