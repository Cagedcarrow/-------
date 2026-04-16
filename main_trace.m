%% =========================================================================
% 论文项目：基于改进RRT算法的抹泥机械臂路径规划系统 (轨迹测试版)
% 脚本名称：main_trace.m
% 修改说明：已移除自碰撞和环境干涉的阻断机制，优先保证轨迹规划与IK联调
% =========================================================================
clc; clear; close all;

% 1. 环境配置：添加模块化文件夹路径
addpath('gui', 'kinematics', 'planner');

% 2. 运动学系统初始化
[robot, ik, ~] = initRobot(); 
endEffector = 'shovel_tip'; % 定义末端执行器参考点

% 3. 加载障碍物 STL 环境
stlFile = '避障场景.STL';
if exist(stlFile, 'file')
    TR = stlread(stlFile);
    points_swapped = TR.Points;
    points_swapped(:, [2, 3]) = points_swapped(:, [3, 2]); 
    
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
    
    fprintf('\n%s\n', repmat('=',1,30));
    fprintf('       模型加载完成\n');
    fprintf('%s\n', repmat('=',1,30));
    fprintf('机械臂(URDF) 尺寸范围: X:[%.2f,%.2f], Y:[%.2f,%.2f], Z:[%.2f,%.2f]\n', ...
            robot_min(1), robot_max(1), robot_min(2), robot_max(2), robot_min(3), robot_max(3));
    fprintf('%s\n', repmat('-',1,30));
else
    error('未找到障碍物 STL 文件: %s', stlFile);
end

% 4. 启动图形化界面 (GUI)
handles = MoveShovelTip(robot, @(src, event) startSimulation(ancestor(src, 'figure')));

% 5. 全局数据持久化
handles.robot = robot;
handles.ik = ik;
handles.endEffector = endEffector;
handles.currentConfig = homeConfiguration(robot); % 初始姿态

% 存储障碍物数据
handles.obstacle.TR = TR;
handles.obstacle.points = points_swapped;

% 在 GUI 中绘制障碍物
h_stl = trisurf(TR.ConnectivityList, points_swapped(:,1), points_swapped(:,2), points_swapped(:,3), ...
    'Parent', handles.ax, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
handles.obstacle.h_stl = h_stl;

% 添加图例和比例尺
legend(h_stl, '环境障碍物', 'Location', 'northeast');
ref_line_start = [robot_min(1), robot_min(2), robot_min(3)];
ref_line_end = ref_line_start + [1, 0, 0]; 
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
title(handles.ax, '【调试模式】干涉检查已关闭');

% 启动基座控制窗口
mainFigHandle = handles.fig;
baseH = BaseControl(@(src, event) updateBasePose(mainFigHandle));
handles.baseH = baseH;
guidata(handles.fig, handles);

fprintf('✅ 轨迹测试系统已就绪。忽略干涉，将强制执行路径规划。\n');

%% =========================================================================
% 核心执行模块：路径规划与避障驱动
% =========================================================================
function startSimulation(figHandle)
    h = guidata(figHandle);
    
    % 1. 获取起点和终点
    [startPos, endPos] = getPathPoints(h);
    if isempty(startPos), return; end 
    
    fprintf('\n=== 路径规划调试信息 ===\n');
    fprintf('1. GUI设置的起点: [%.3f, %.3f, %.3f]\n', startPos);
    fprintf('2. GUI设置的终点: [%.3f, %.3f, %.3f]\n', endPos);
    
    numSteps = 60;
    pathPoints = linearInterpolator(startPos, endPos, numSteps);
    
    % 2. 初始化轨迹显示与参数
    if isfield(h, 'hTrail') && ishandle(h.hTrail), delete(h.hTrail); end
    h.hTrail = plot3(h.ax, NaN, NaN, NaN, 'b-', 'LineWidth', 2);
    title(h.ax, '正在强制执行路径追踪...', 'Color', 'b');
    
    weights = [0.1 0.1 0.1 1 1 1]; 
    fixedOrientation = eul2tform([0, 0, 0], 'ZYX'); 
    
    trailData = [];
    currentConfig = h.currentConfig;
    
    % 3. 路径执行循环 (移除了碰撞阻断机制)
    for i = 1:numSteps
        targetXYZ = pathPoints(i, :);
        targetTform = trvec2tform(targetXYZ) * fixedOrientation;
        
        % 逆运动学求解
        [newConfig, ikSuccess] = solveIK(h.ik, h.endEffector, targetTform, weights, currentConfig);
        
        if ikSuccess
            actualTform = getTransform(h.robot, newConfig, h.endEffector);
            actualXYZ = tform2trvec(actualTform);
            
            % --- 干涉仅作视觉警告，不阻断运动 ---
            obstacle_points = h.obstacle.points;
            distances = sqrt(sum((obstacle_points - actualXYZ).^2, 2));
            min_distance = min(distances);
            
            if min_distance < 0.01 
                % 仅在发生碰撞时在终端提示，并打上红色叉号，但程序继续运行
                fprintf('警告: 路径点 %d 发生穿模干涉，强制通过。\n', i);
                plot3(h.ax, actualXYZ(1), actualXYZ(2), actualXYZ(3), 'rx', ...
                      'MarkerSize', 10, 'LineWidth', 2);
            end
            
            % --- 强制更新关节配置并绘图 ---
            currentConfig = newConfig;
            
            % 渲染机器人与运动轨迹
            show(h.robot, currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
            
            trailData = [trailData; actualXYZ]; %#ok<AGROW>
            set(h.hTrail, 'XData', trailData(:,1), 'YData', trailData(:,2), 'ZData', trailData(:,3));
            
            drawnow; % 确保动画实时性
        else
            warning('路径点 %d 逆运动学无解，可能是目标点超出工作空间。', i);
            break; % IK无解时必须停下，否则无法绘图
        end
    end
    
    % 4. 更新最终状态
    h.currentConfig = currentConfig;
    guidata(figHandle, h);
    title(h.ax, '✅ 轨迹追踪测试完成');
    fprintf('=== 轨迹执行结束 ===\n\n');
end

%% =========================================================================
% 辅助函数：从GUI获取起点和终点坐标
% =========================================================================
function [startPos, endPos] = getPathPoints(h)
    if isfield(h, 'startSliders') && isfield(h, 'endSliders')
        startPos = zeros(1, 3);
        for i = 1:3
            if ishandle(h.startSliders{i})
                startPos(i) = h.startSliders{i}.Value;
            else
                startPos(i) = 0.5; 
            end
        end
        endPos = zeros(1, 3);
        for i = 1:3
            if ishandle(h.endSliders{i})
                endPos(i) = h.endSliders{i}.Value;
            else
                endPos(i) = 0.5; 
            end
        end
    else
        startPos = [0.5, 0, 0.5];
        endPos = [0.5, 0.5, 0.5];
    end
end

%% =========================================================================
% 基座控制回调函数
% =========================================================================
function updateBasePose(mainFig)
    h = guidata(mainFig);
    hb = guidata(gcf); 
    if isempty(h) || isempty(hb), return; end

    bx = hb.sliders{1}.Value;
    by = hb.sliders{2}.Value;
    bz = hb.sliders{3}.Value;
    br = deg2rad(hb.sliders{4}.Value);
    bp = deg2rad(hb.sliders{5}.Value);
    byaw = deg2rad(hb.sliders{6}.Value);
    
    for k=1:6, hb.txtVals{k}.String = sprintf('%.2f', hb.sliders{k}.Value); end

    tformBase = trvec2tform([bx, by, bz]) * eul2tform([byaw, bp, br], 'ZYX');
    rootBody = h.robot.Bodies{1}; 
    setFixedTransform(rootBody.Joint, tformBase);

    show(h.robot, h.currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
    
    if isfield(h, 'hStart') && ishandle(h.hStart)
        [startPos, ~] = getPathPoints(h);
        set(h.hStart, 'XData', startPos(1), 'YData', startPos(2), 'ZData', startPos(3));
    end
    
    if isfield(h, 'hEnd') && ishandle(h.hEnd)
        [~, endPos] = getPathPoints(h);
        set(h.hEnd, 'XData', endPos(1), 'YData', endPos(2), 'ZData', endPos(3));
    end
    
    drawnow limitrate;
end