%% =========================================================================
% 论文项目：基于改进RRT算法的抹泥机械臂路径规划系统
% 主执行程序 (Main Entry Point)
% 功能：基于 main_trace.m 重写，保留稳定轨迹执行，并增加简单的"铲子 vs 本体"检测
% 说明：仅使用 MATLAB 自带 checkCollision 结果，筛选"铲子"与本体的碰撞，不做复杂 mesh 重建
% =========================================================================
clc; clear; close all;

% 1. 环境配置
addpath('gui', 'kinematics', 'planner');

% 2. 运动学系统初始化
[robot, ik, ~] = initRobot();
endEffector = 'shovel_tip';

% 3. 加载障碍物 STL 环境
stlFile = '避障场景.STL';
if exist(stlFile, 'file')
    TR = stlread(stlFile);
    points_swapped = TR.Points;
    points_swapped(:, [2, 3]) = points_swapped(:, [3, 2]);

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

% 4. 启动图形化界面
handles = MoveShovelTip(robot, @(src, event) startSimulation(ancestor(src, 'figure')));

% 5. 全局数据持久化
handles.robot = robot;
handles.ik = ik;
handles.endEffector = endEffector;
handles.currentConfig = homeConfiguration(robot);
handles.basePoseWorld = eye(4); % 基座在世界坐标系中的统一状态源
handles.obstacle.TR = TR;
handles.obstacle.points = points_swapped;

h_stl = trisurf(TR.ConnectivityList, points_swapped(:,1), points_swapped(:,2), points_swapped(:,3), ...
    'Parent', handles.ax, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
handles.obstacle.h_stl = h_stl;

legend(h_stl, '环境障碍物', 'Location', 'northeast');
ref_line_start = [robot_min(1), robot_min(2), robot_min(3)];
ref_line_end = ref_line_start + [1, 0, 0];
plot3(handles.ax, [ref_line_start(1), ref_line_end(1)], ...
      [ref_line_start(2), ref_line_end(2)], ...
      [ref_line_start(3), ref_line_end(3)], ...
      'r-', 'LineWidth', 2);
text(handles.ax, ref_line_end(1), ref_line_end(2), ref_line_end(3), '1m', ...
     'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);

view(handles.ax, 135, 30);
grid(handles.ax, 'on');
axis(handles.ax, 'equal');
camlight(handles.ax);
lighting(handles.ax, 'gouraud');
title(handles.ax, '机械臂路径规划与铲子防自碰撞系统');

% 6. 启动基座控制窗口
mainFigHandle = handles.fig;
baseH = BaseControl(@(src, event) updateBasePose(mainFigHandle));
handles.baseH = baseH;
guidata(handles.fig, handles);

fprintf('✅ 系统已就绪。将执行轨迹规划，并仅检测铲子与本体的碰撞。\n');

%% =========================================================================
% 核心执行模块：路径规划与简化碰撞检测
% =========================================================================
function startSimulation(figHandle)
    h = guidata(figHandle);
    if ~isfield(h, 'basePoseWorld')
        h.basePoseWorld = eye(4);
    end
    
    % 6. 防止重复启动导致回调交叉
    if isfield(h, 'isRunning') && h.isRunning
        fprintf('⚠️ 已有任务正在运行，本次请求被忽略。\n');
        return;
    end
    h.isRunning = true;
    guidata(figHandle, h);

    [startPos, endPos] = getPathPoints(h);
    if isempty(startPos)
        h.isRunning = false;
        guidata(figHandle, h);
        return; 
    end

    fprintf('\n=== 路径规划调试信息 ===\n');
    fprintf('1. GUI设置的起点: [%.3f, %.3f, %.3f]\n', startPos);
    fprintf('2. GUI设置的终点: [%.3f, %.3f, %.3f]\n', endPos);
    
    % 4. 坐标系统一：打印基座变换信息
    baseTform = h.basePoseWorld;
    fprintf('3. 基座变换矩阵:\n');
    disp(baseTform);
    
    % 先定义执行状态，规划与执行都复用
    weights = [0.1 0.1 0.1 1 1 1];
    currentConfig = h.currentConfig;
    
    % 规划与执行统一姿态策略：使用当前末端姿态作为共享目标姿态
    currentEEAtPlan = getTransform(h.robot, currentConfig, h.endEffector);
    sharedOrientation = rotm2tform(tform2rotm(currentEEAtPlan));

    % 使用 DP-RRT 进行外部 STL 障碍避障规划
    fprintf('4. 正在调用 DP-RRT 进行路径规划...\n');
    plannerParams = struct();
    plannerParams.maxIter = 3000;
    plannerParams.goalRadius = 0.04;
    plannerParams.envSafeDist = 0.02;
    plannerParams.edgeSampleStep = 0.02;
    plannerParams.enableRobotCheck = true;  % 开启规划阶段机器人可达性与自碰撞检查
    plannerParams.enableEnvCheck = true;
    plannerParams.targetOrientation = sharedOrientation;

    [pathPoints, planInfo] = rrtPlanner( ...
        startPos, endPos, h.robot, h.ik, weights, currentConfig, h.obstacle.points, plannerParams);

    if isempty(pathPoints)
        warning('DP-RRT 规划失败，回退到直线插值路径。');
        numSteps = 60;
        pathPoints = linearInterpolator(startPos, endPos, numSteps);
        fprintf('5. 规划结果: 回退直线插值，共 %d 个路径点。\n', size(pathPoints,1));
    else
        fprintf('5. 规划结果: DP-RRT 成功，路径点=%d, 树节点=%d, 用时=%.3fs\n', ...
            size(pathPoints,1), size(planInfo.tree,1), planInfo.time);
    end

    % 7. RRT 输出路径过短时的处理
    if size(pathPoints,1) <= 2
        fprintf('⚠️ 路径点过少(%d个)，将验证起点IK...\n', size(pathPoints,1));
        % 验证起点IK
        startTform = trvec2tform(startPos) * sharedOrientation;
        [startConfig, startSuccess] = solveIK(h.ik, h.endEffector, startTform, weights, currentConfig);
        if startSuccess
            actualTform = getTransform(h.robot, startConfig, h.endEffector);
            actualXYZ = tform2trvec(actualTform);
            positionError = norm(startPos - actualXYZ);
            fprintf('起点IK误差: %.4f m\n', positionError);
        end
    end

    % 1. 细分路径点，使动画更平滑
    playbackStep = 0.01;   % 越小越慢越平滑
    rawCount = size(pathPoints,1);
    pathPoints = densifyPath(pathPoints, playbackStep);
    fprintf('6. 路径点: 原始=%d, 细分后=%d\n', rawCount, size(pathPoints,1));

    if isfield(h, 'hTrail') && ishandle(h.hTrail), delete(h.hTrail); end
    h.hTrail = plot3(h.ax, NaN, NaN, NaN, 'b-', 'LineWidth', 2);
    title(h.ax, '正在执行路径追踪与铲子防撞检测...', 'Color', 'b');
    
    % 2. 调整动画速度控制参数
    % 数值越大动画越慢，建议范围 0.02 ~ 0.12
    framePause = 0.03;  % 从 0.01 调整为 0.03

    % 规划与执行使用相同姿态策略（与 plannerParams.targetOrientation 对齐）
    mode = 'planner_aligned';
    
    % IK 位置误差容限（放宽到 5~8cm 区间）
    ikPosTol = 0.06;  % 6cm 误差容限

    trailData = [];
    hasCollision = false;
    consecutiveFailures = 0;
    maxConsecutiveFailures = 5;

    for i = 1:size(pathPoints, 1)
        % 5. 修复 h.ax 失效崩溃
        if ~isfield(h, 'ax') || ~ishandle(h.ax) || ~strcmp(get(h.ax, 'Type'), 'axes')
            warning('主坐标轴句柄失效，停止本次执行。');
            break;
        end
        
        targetXYZ = pathPoints(i, :);
        
        % 2. 目标姿态策略
        if strcmp(mode, 'planner_aligned')
            targetTform = trvec2tform(targetXYZ) * sharedOrientation;
        else
            targetTform = trvec2tform(targetXYZ) * sharedOrientation;
        end

        % 3. IK 多初值兜底
        ikSeeds = {currentConfig, homeConfiguration(h.robot)};
        if i > 1 && isfield(h, 'lastValidConfig')
            ikSeeds{end+1} = h.lastValidConfig;
        end
        
        bestConfig = [];
        bestError = inf;
        bestSuccess = false;
        
        for seedIdx = 1:length(ikSeeds)
            [newConfig, ikSuccess] = solveIK(h.ik, h.endEffector, targetTform, weights, ikSeeds{seedIdx});
            
            if ikSuccess
                actualTform = getTransform(h.robot, newConfig, h.endEffector);
                actualXYZ = tform2trvec(actualTform);
                positionError = norm(targetXYZ - actualXYZ);
                
                % 1. IK 成功判据改成"双门限"
                if positionError <= ikPosTol
                    % 检查铲子碰撞
                    [shovelHitBody, ~] = checkShovelBodyCollision(h.robot, newConfig, h.basePoseWorld);
                    if ~shovelHitBody && positionError < bestError
                        bestConfig = newConfig;
                        bestError = positionError;
                        bestSuccess = true;
                    end
                end
            end
        end
        
        if ~bestSuccess
            % 所有种子都失败
            warning('路径点 %d 所有IK种子均失败或误差过大', i);
            consecutiveFailures = consecutiveFailures + 1;
            
            if consecutiveFailures >= maxConsecutiveFailures
                fprintf('连续 %d 个点IK失败，终止执行。\n', maxConsecutiveFailures);
                break;
            end
            continue;
        end
        
        newConfig = bestConfig;
        actualTform = getTransform(h.robot, newConfig, h.endEffector);
        actualXYZ = tform2trvec(actualTform);
        positionError = bestError;
        
        consecutiveFailures = 0;  % 重置失败计数
        
        if i <= 5
            fprintf('点 %d: 目标[%.3f,%.3f,%.3f] -> 实际[%.3f,%.3f,%.3f] 误差:%.4f\n', ...
                i, targetXYZ, actualXYZ, positionError);
        end

        [shovelHitBody, hitInfo] = checkShovelBodyCollision(h.robot, newConfig, h.basePoseWorld);

        if i <= 5
            fprintf('   铲子检测: 硬碰撞=%d, 对象=%s, 判定=%s, 最小距离=%.4f\n', ...
                shovelHitBody, hitInfo.body_b, hitInfo.method, hitInfo.min_distance);
            if hitInfo.proximity_warning
                fprintf('   铲子接近告警: 对象=%s, 距离=%.4f m\n', ...
                    hitInfo.warning_body, hitInfo.warning_distance);
            end
        end

        if shovelHitBody
            hasCollision = true;
            title(h.ax, '❌ 警告：铲子与机械臂本体发生干涉！已停止', 'Color', 'r');
            plot3(h.ax, actualXYZ(1), actualXYZ(2), actualXYZ(3), 'rx', ...
                'MarkerSize', 12, 'LineWidth', 2);

            fprintf('\n=== 铲子碰撞检测结果 ===\n');
            fprintf('路径点 %d 检测到铲子与本体碰撞\n', i);
            fprintf('碰撞对象 A: %s\n', hitInfo.body_a);
            fprintf('碰撞对象 B: %s\n', hitInfo.body_b);
            fprintf('判定方法: %s\n', hitInfo.method);
            fprintf('最小距离: %.6f m\n', hitInfo.min_distance);
            fprintf('当前关节角: %s\n', mat2str(newConfig, 4));
            fprintf('========================\n\n');
            break;
        end

        obstacle_points = h.obstacle.points;
        distances = sqrt(sum((obstacle_points - actualXYZ).^2, 2));
        min_distance = min(distances);

        if min_distance < 0.01
            fprintf('警告: 路径点 %d 末端接近环境障碍物，强制通过。\n', i);
            plot3(h.ax, actualXYZ(1), actualXYZ(2), actualXYZ(3), 'yo', ...
                  'MarkerSize', 8, 'LineWidth', 2);
        end

        currentConfig = newConfig;
        h.lastValidConfig = newConfig;  % 保存成功配置

        show(h.robot, currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
        redrawEnvironment(h);

        trailData = [trailData; actualXYZ]; %#ok<AGROW>
        set(h.hTrail, 'XData', trailData(:,1), 'YData', trailData(:,2), 'ZData', trailData(:,3));

        drawnow;
        pause(framePause);  % 使用调整后的暂停时间
    end

    h.currentConfig = currentConfig;
    h.isRunning = false;  % 释放运行锁
    if isfield(h, 'lastValidConfig')
        h = rmfield(h, 'lastValidConfig');
    end
    guidata(figHandle, h);

    if ~hasCollision
        title(h.ax, '✅ 轨迹追踪完成');
    end
    fprintf('=== 轨迹执行结束 ===\n\n');
end

%% =========================================================================
% 简单检测：仅关注铲子是否与机械臂本体碰撞
% =========================================================================
function [isColliding, info] = checkShovelBodyCollision(robot, config, basePoseWorld)
    info = struct( ...
        'body_a', '铲子', ...
        'body_b', '', ...
        'method', 'none', ...
        'min_distance', inf, ...
        'proximity_warning', false, ...
        'warning_body', '', ...
        'warning_distance', inf);
    isColliding = false;

    monitoredBodies = ["base", "ur10_shoulder", "ur10_upper_arm", ...
        "ur10_forearm", "ur10_wrist_1", "ur10_wrist_2"];

    [robotCollision, details] = checkCollision(robot, config, ...
        'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');

    bodyLabels = ["base", string(robot.BodyNames)];
    shovelIdx = getBodyIndexByName(bodyLabels, "铲子");
    if robotCollision && ~isempty(shovelIdx)
        for i = 1:numel(monitoredBodies)
            bodyName = monitoredBodies(i);
            bodyIdx = getBodyIndexByName(bodyLabels, bodyName);
            if isempty(bodyIdx)
                continue;
            end

            pairValue = details(shovelIdx, bodyIdx);
            reverseValue = details(bodyIdx, shovelIdx);

            pairDist = extractFiniteDistance(pairValue, reverseValue);
            if pairDist < info.min_distance
                info.min_distance = pairDist;
                info.body_b = char(bodyName);
            end

            if isCollisionEntry(pairValue) || isCollisionEntry(reverseValue)
                isColliding = true;
                info.body_b = char(bodyName);
                info.method = 'checkCollision';
                if ~isfinite(info.min_distance)
                    info.min_distance = 0;
                end
                return;
            end
        end
    end

    [proximityHit, proximityInfo] = checkShovelBodyProximity(robot, config, monitoredBodies, basePoseWorld);
    if proximityInfo.min_distance < info.min_distance
        info.min_distance = proximityInfo.min_distance;
        info.body_b = proximityInfo.body_b;
    end

    if proximityHit
        info.proximity_warning = true;
        info.warning_body = proximityInfo.body_b;
        info.warning_distance = proximityInfo.min_distance;
        if strcmp(info.method, 'none')
            info.method = 'proximity_warning';
            info.body_b = proximityInfo.body_b;
            info.min_distance = proximityInfo.min_distance;
        end
    end
end

%% =========================================================================
% 判断 checkCollision 返回矩阵中的单元是否表示实际碰撞
% =========================================================================
function tf = isCollisionEntry(value)
    tf = false;

    if isempty(value)
        return;
    end

    if isnan(value)
        tf = true;
        return;
    end

    % 某些版本/几何组合下，深度碰撞也可能表现为非正值
    if isfinite(value) && value <= 0
        tf = true;
    end
end

%% =========================================================================
% 更简单的铲子近距离检测：采样铲子轴线点到本体关键连杆原点的距离
% =========================================================================
function [isColliding, info] = checkShovelBodyProximity(robot, config, monitoredBodies, basePoseWorld)
    isColliding = false;
    info = struct('body_b', '', 'min_distance', inf);

    shovelTform = getTransform(robot, config, '铲子');
    tipTform = getTransform(robot, config, 'shovel_tip');
    shovelOrigin = tform2trvec(shovelTform);
    shovelTip = tform2trvec(tipTform);

    sampleRatios = linspace(0, 1, 5)';
    shovelSamples = shovelOrigin + sampleRatios .* (shovelTip - shovelOrigin);

    for i = 1:numel(monitoredBodies)
        bodyName = monitoredBodies(i);
        if bodyName == "base"
            if nargin >= 4 && ~isempty(basePoseWorld)
                bodyPoint = basePoseWorld(1:3, 4)';
            else
                bodyPoint = [0, 0, 0];
            end
        else
            bodyPoint = tform2trvec(getTransform(robot, config, char(bodyName)));
        end

        distances = vecnorm(shovelSamples - bodyPoint, 2, 2);
        minDist = min(distances);

        threshold = getBodySafetyThreshold(bodyName);
        if minDist < info.min_distance
            info.min_distance = minDist;
            info.body_b = char(bodyName);
        end

        if minDist < threshold
            isColliding = true;
            return;
        end
    end
end

%% =========================================================================
% 稳定名称索引，避免不同字符串类型造成索引错配
% =========================================================================
function idx = getBodyIndexByName(bodyLabels, bodyName)
    idx = find(bodyLabels == string(bodyName), 1);
end

%% =========================================================================
% 提取 checkCollision 返回的有限距离
% =========================================================================
function dist = extractFiniteDistance(a, b)
    dist = inf;
    candidates = [a, b];
    for i = 1:numel(candidates)
        value = candidates(i);
        if ~isempty(value) && isfinite(value) && ~isnan(value)
            dist = min(dist, value);
        end
    end
end

%% =========================================================================
% 各关键连杆的简单安全距离阈值
% =========================================================================
function threshold = getBodySafetyThreshold(bodyName)
    switch char(bodyName)
        case 'base'
            threshold = 0.10;
        case 'ur10_shoulder'
            threshold = 0.08;
        case 'ur10_upper_arm'
            threshold = 0.07;
        case 'ur10_forearm'
            threshold = 0.06;
        case {'ur10_wrist_1', 'ur10_wrist_2'}
            threshold = 0.045;
        otherwise
            threshold = 0.05;
    end
end

%% =========================================================================
% 重绘环境
% =========================================================================
function redrawEnvironment(h)
    if isfield(h, 'obstacle') && isfield(h.obstacle, 'TR')
        trisurf(h.obstacle.TR.ConnectivityList, ...
            h.obstacle.points(:,1), h.obstacle.points(:,2), h.obstacle.points(:,3), ...
            'Parent', h.ax, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    end
end

% 辅助函数：从 GUI 获取起点和终点坐标
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

function updateBasePose(mainFig)
    % 5. 修复 h.ax 失效崩溃
    if ~ishandle(mainFig)
        return;
    end
    
    h = guidata(mainFig);
    hb = guidata(gcf);
    if isempty(h) || isempty(hb), return; end

    bx = hb.sliders{1}.Value;
    by = hb.sliders{2}.Value;
    bz = hb.sliders{3}.Value;
    br = deg2rad(hb.sliders{4}.Value);
    bp = deg2rad(hb.sliders{5}.Value);
    byaw = deg2rad(hb.sliders{6}.Value);

    for k = 1:6
        hb.txtVals{k}.String = sprintf('%.2f', hb.sliders{k}.Value);
    end

    tformBase = trvec2tform([bx, by, bz]) * eul2tform([byaw, bp, br], 'ZYX');
    rootBody = h.robot.Bodies{1};
    setFixedTransform(rootBody.Joint, tformBase);
    h.basePoseWorld = tformBase;
    guidata(mainFig, h);

    % 5. 修复 h.ax 失效崩溃
    if ~isfield(h, 'ax') || ~ishandle(h.ax) || ~strcmp(get(h.ax, 'Type'), 'axes')
        warning('主坐标轴句柄失效，跳过重绘。');
        return;
    end
    
    show(h.robot, h.currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
    redrawEnvironment(h);

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

%% =========================================================================
% 路径细分函数：将稀疏路径点插值为密集路径点
% =========================================================================
function densePath = densifyPath(pathPoints, stepLen)
    % 如果路径点少于2个，直接返回
    if size(pathPoints,1) <= 1
        densePath = pathPoints;
        return;
    end
    
    % 初始化结果路径，从第一个点开始
    densePath = pathPoints(1,:);
    
    % 遍历每一段路径
    for i = 1:size(pathPoints,1)-1
        p1 = pathPoints(i,:);
        p2 = pathPoints(i+1,:);
        
        % 计算线段长度
        seg = p2 - p1;
        L = norm(seg);
        
        % 根据步长计算需要插入的点数
        n = max(1, ceil(L / max(stepLen, 1e-4)));
        
        % 线性插值
        t = (1:n)' / n;
        pts = p1 + t .* seg;
        
        % 添加到结果中（跳过第一个点，避免重复）
        densePath = [densePath; pts(2:end,:)]; %#ok<AGROW>
    end
end
