%% =========================================================================
% 机械臂仅本体研究版本（无外部 STL 障碍）
% 文件：main_ur_control.m
% 目标：
%   1) 只研究机械臂本体*运动与末端轨迹
%   2) 支持基座世界坐标变换
%   3) 保留“铲子 vs 本体”自碰撞检查
% =========================================================================
clc; clear; close all;

addpath('gui', 'kinematics', 'planner');

% 1) 初始化机器人与 IK
[robot, ik, ~] = initWorldRobot();
endEffector = 'shovel_tip';

% 2) 启动主界面
handles = MoveShovelTip(robot, @(src, event) startSimulation(ancestor(src, 'figure')));

% 3) 全局状态
handles.robot = robot;
handles.ik = ik;
handles.endEffector = endEffector;
handles.currentConfig = homeConfiguration(robot);
handles.basePoseWorld = eye(4); % 统一基座世界位姿

view(handles.ax, 135, 30);
grid(handles.ax, 'on');
axis(handles.ax, 'equal');
setFixedWorkspace(handles.ax);
camlight(handles.ax);
lighting(handles.ax, 'gouraud');
title(handles.ax, 'UR10 机械臂本体研究（世界基座控制）');

% 4) 启动基座控制窗口
mainFigHandle = handles.fig;
baseH = BaseControl(@(src, event) updateBasePose(mainFigHandle, src));
handles.baseH = baseH;
guidata(handles.fig, handles);

fprintf('系统已就绪（无外部 STL 障碍）。\n');

%% =========================================================================
% 轨迹规划与执行
% =========================================================================
function startSimulation(figHandle)
    h = guidata(figHandle);
    if isempty(h)
        return;
    end
    if ~isfield(h, 'basePoseWorld')
        h.basePoseWorld = eye(4);
    end

    % 防止重复启动
    if isfield(h, 'isRunning') && h.isRunning
        fprintf('已有任务在运行，本次请求忽略。\n');
        return;
    end
    h.isRunning = true;
    h = rebuildIKSolver(h);
    guidata(figHandle, h);

    [startPos, endPos] = getPathPoints(h);
    if isempty(startPos)
        h.isRunning = false;
        guidata(figHandle, h);
        return;
    end

    fprintf('\n=== 机械臂本体轨迹调试信息 ===\n');
    fprintf('1. 起点: [%.3f, %.3f, %.3f]\n', startPos);
    fprintf('2. 终点: [%.3f, %.3f, %.3f]\n', endPos);
    fprintf('3. 基座变换矩阵:\n');
    disp(h.basePoseWorld);
    eeNowXYZ = tform2trvec(getTransform(h.robot, h.currentConfig, h.endEffector));
    fprintf('4. 当前末端世界坐标: [%.3f, %.3f, %.3f]\n', eeNowXYZ);
    fprintf('5. IK求解器已同步到当前基座。\n');

    weights = [0.1 0.1 0.1 1 1 1];
    currentConfig = h.currentConfig;

    % 规划与执行统一姿态：使用当前末端姿态
    eeNow = getTransform(h.robot, currentConfig, h.endEffector);
    sharedOrientation = rotm2tform(tform2rotm(eeNow));

    % 使用 DP-RRT（无外部障碍）
    plannerParams = struct();
    plannerParams.maxIter = 2500;
    plannerParams.goalRadius = 0.04;
    plannerParams.enableEnvCheck = false;
    plannerParams.enableRobotCheck = true;
    plannerParams.targetOrientation = sharedOrientation;

    fprintf('6. 调用 DP-RRT 规划（仅本体约束）...\n');
    fprintf('   规划中，请等待...\n');
    [pathPoints, planInfo] = rrtPlanner( ...
        startPos, endPos, h.robot, h.ik, weights, currentConfig, [], plannerParams);

    if isempty(pathPoints)
        warning('DP-RRT 失败，回退直线插值。');
        pathPoints = linearInterpolator(startPos, endPos, 80);
        fprintf('7. 回退直线插值，路径点=%d\n', size(pathPoints, 1));
    else
        fprintf('7. DP-RRT 成功，路径点=%d，树节点=%d，用时=%.3fs\n', ...
            size(pathPoints, 1), size(planInfo.tree, 1), planInfo.time);
    end

    % 路径细分，增强动画可读性
    playbackStep = 0.01;
    rawCount = size(pathPoints, 1);
    pathPoints = densifyPath(pathPoints, playbackStep);
    fprintf('8. 路径点: 原始=%d, 细分后=%d\n', rawCount, size(pathPoints, 1));

    if isfield(h, 'hTrail') && ishandle(h.hTrail)
        delete(h.hTrail);
    end
    h.hTrail = plot3(h.ax, NaN, NaN, NaN, 'b-', 'LineWidth', 2);
    title(h.ax, '执行轨迹中（仅本体约束）', 'Color', 'b');
    [startPosNow, endPosNow] = getPathPoints(h);
    ensureStartEndMarkers(h, startPosNow, endPosNow);

    framePause = 0.03;
    hasCollision = false;
    trailData = [];

    for i = 1:size(pathPoints, 1)
        if ~isfield(h, 'ax') || ~ishandle(h.ax) || ~strcmp(get(h.ax, 'Type'), 'axes')
            warning('主坐标轴句柄失效，停止执行。');
            break;
        end

        targetXYZ = pathPoints(i, :);
        targetTform = trvec2tform(targetXYZ) * sharedOrientation;

        % 多初值 IK
        ikSeeds = {currentConfig, homeConfiguration(h.robot)};
        if i > 1 && isfield(h, 'lastValidConfig')
            ikSeeds{end+1} = h.lastValidConfig;
        end

        bestConfig = [];
        bestError = inf;
        for s = 1:numel(ikSeeds)
            [cfgTry, ok] = solveIK(h.ik, h.endEffector, targetTform, weights, ikSeeds{s});
            if ~ok
                continue;
            end
            eeTry = getTransform(h.robot, cfgTry, h.endEffector);
            xyzTry = tform2trvec(eeTry);
            err = norm(targetXYZ - xyzTry);
            if err < bestError
                bestError = err;
                bestConfig = cfgTry;
            end
        end

        if isempty(bestConfig)
            warning('路径点 %d IK失败，跳过该点继续执行。', i);
            continue;
        end

        newConfig = bestConfig;
        eeTform = getTransform(h.robot, newConfig, h.endEffector);
        actualXYZ = tform2trvec(eeTform);

        if i <= 5
            fprintf('点 %d IK误差: %.4f m\n', i, bestError);
        end

        [shovelHitBody, hitInfo] = checkMotionCollision(h.robot, currentConfig, newConfig, h.basePoseWorld);
        if shovelHitBody
            hasCollision = true;
            title(h.ax, '检测到铲子与本体碰撞，已停止', 'Color', 'r');
            plot3(h.ax, actualXYZ(1), actualXYZ(2), actualXYZ(3), 'rx', ...
                'MarkerSize', 12, 'LineWidth', 2);
            fprintf('碰撞点 %d: %s vs %s, 检测方式=%s, 最小距离=%.4f m, 插值步=%d\n', ...
                i, hitInfo.body_a, hitInfo.body_b, hitInfo.method, hitInfo.min_distance, hitInfo.sample_index);
            break;
        end

        currentConfig = newConfig;
        h.lastValidConfig = newConfig;

        show(h.robot, currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
        setFixedWorkspace(h.ax);
        [startPosNow, endPosNow] = getPathPoints(h);
        ensureStartEndMarkers(h, startPosNow, endPosNow);
        trailData = [trailData; actualXYZ]; %#ok<AGROW>
        set(h.hTrail, 'XData', trailData(:,1), 'YData', trailData(:,2), 'ZData', trailData(:,3));
        drawnow;
        pause(framePause);
    end

    h.currentConfig = currentConfig;
    h.isRunning = false;
    if isfield(h, 'lastValidConfig')
        h = rmfield(h, 'lastValidConfig');
    end
    guidata(figHandle, h);

    if ~hasCollision
        title(h.ax, '轨迹执行结束');
    end
    fprintf('=== 执行结束 ===\n\n');
end

%% =========================================================================
% 仅检查“铲子 vs 本体关键连杆”
% =========================================================================
function [isColliding, info] = checkShovelBodyCollision(robot, config, basePoseWorld)
    %#ok<INUSD>
    info = struct('body_a', '铲子', 'body_b', '', 'method', 'none', 'min_distance', inf);
    isColliding = false;

    [robotCollision, details] = checkCollision(robot, config, ...
        'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');
    if ~robotCollision
        return;
    end

    bodyLabels = getCollisionBodyLabels(robot, details);
    shovelIdx = find(bodyLabels == "铲子", 1);
    if isempty(shovelIdx)
        return;
    end

    monitoredBodies = ["ur10", "ur10_shoulder", "ur10_upper_arm", ...
        "ur10_forearm", "ur10_wrist_1", "ur10_wrist_2"];

    for i = 1:numel(monitoredBodies)
        bodyIdx = find(bodyLabels == monitoredBodies(i), 1);
        if isempty(bodyIdx)
            continue;
        end
        pairValue = details(shovelIdx, bodyIdx);
        reverseValue = details(bodyIdx, shovelIdx);
        pairDist = extractFiniteDistance(pairValue, reverseValue);
        if pairDist < info.min_distance
            info.min_distance = pairDist;
            info.body_b = char(monitoredBodies(i));
        end
        if isCollisionEntry(pairValue) || isCollisionEntry(reverseValue)
            isColliding = true;
            info.method = 'checkCollision';
            if ~isfinite(info.min_distance)
                info.min_distance = 0;
            end
            return;
        end
    end

    % 额外近距离告警（不作为硬碰撞）
    [~, proximityInfo] = checkShovelBodyProximity(robot, config, monitoredBodies, basePoseWorld);
    if proximityInfo.min_distance < info.min_distance
        info.min_distance = proximityInfo.min_distance;
        info.body_b = proximityInfo.body_b;
        info.method = 'proximity_warning';
    end
end

function [isColliding, info] = checkMotionCollision(robot, configA, configB, basePoseWorld)
    info = struct('body_a', '铲子', 'body_b', '', 'method', 'none', ...
        'min_distance', inf, 'sample_index', 0);
    isColliding = false;

    delta = configB - configA;
    maxJointStep = deg2rad(2.0);
    numSamples = max(2, ceil(max(abs(delta)) / maxJointStep) + 1);

    for k = 1:numSamples
        alpha = (k - 1) / max(numSamples - 1, 1);
        cfg = configA + alpha * delta;
        [hitNow, hitInfo] = checkShovelBodyCollision(robot, cfg, basePoseWorld);

        if hitInfo.min_distance < info.min_distance
            info = hitInfo;
            info.sample_index = k;
        end

        if hitNow
            isColliding = true;
            info = hitInfo;
            info.sample_index = k;
            if isempty(info.method) || strcmp(info.method, 'none')
                info.method = 'swept_checkCollision';
            else
                info.method = ['swept_' info.method];
            end
            return;
        end
    end
end

function tf = isCollisionEntry(value)
    tf = false;
    if isempty(value)
        return;
    end
    if isnan(value)
        tf = true;
        return;
    end
    if isfinite(value) && value <= 0
        tf = true;
    end
end

function [isColliding, info] = checkShovelBodyProximity(robot, config, monitoredBodies, basePoseWorld)
    %#ok<INUSD>
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
        bodyPoint = tform2trvec(getTransform(robot, config, char(bodyName)));

        distances = vecnorm(shovelSamples - bodyPoint, 2, 2);
        minDist = min(distances);
        if minDist < info.min_distance
            info.min_distance = minDist;
            info.body_b = char(bodyName);
        end
    end
end

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

function bodyLabels = getCollisionBodyLabels(robot, details)
    baseLabel = "base";
    if isprop(robot, 'BaseName') && ~isempty(robot.BaseName)
        baseLabel = string(robot.BaseName);
    end

    bodyLabels = [baseLabel, string(robot.BodyNames)];
    if nargin >= 2 && size(details, 1) ~= numel(bodyLabels)
        bodyLabels = string(robot.BodyNames);
    end
end

function [robot, ik, weights] = initWorldRobot()
    urdfFile = 'ur10_world.urdf';
    if ~exist(urdfFile, 'file')
        error('找不到文件: %s', urdfFile);
    end

    robot = importrobot(urdfFile);
    robot.DataFormat = 'row';
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [1 1 1 1 1 1];

    fprintf('--> [Kinematics] 机器人模型已成功从 "%s" 加载。\n', urdfFile);
    fprintf('--> [Kinematics] 包含 %d 个刚体，IK 求解器已准备就绪。\n', robot.NumBodies);
end

function h = rebuildIKSolver(h)
    h.ik = inverseKinematics('RigidBodyTree', h.robot);
end

%% =========================================================================
% 从 GUI 获取起终点
% =========================================================================
function [startPos, endPos] = getPathPoints(h)
    if isfield(h, 'startSliders') && isfield(h, 'endSliders')
        startPos = zeros(1, 3);
        endPos = zeros(1, 3);
        for i = 1:3
            if ishandle(h.startSliders{i}), startPos(i) = h.startSliders{i}.Value; else, startPos(i) = 0.5; end
            if ishandle(h.endSliders{i}),   endPos(i)   = h.endSliders{i}.Value;   else, endPos(i)   = 0.5; end
        end
    else
        startPos = [0.5, 0, 0.5];
        endPos = [0.5, 0.5, 0.5];
    end
end

%% =========================================================================
% 基座控制回调（世界坐标）
% =========================================================================
function updateBasePose(mainFig, src)
    if ~ishandle(mainFig)
        return;
    end
    h = guidata(mainFig);
    baseFig = [];
    if nargin >= 2 && ~isempty(src) && ishandle(src)
        baseFig = ancestor(src, 'figure');
    end
    if isempty(baseFig) || ~ishandle(baseFig)
        if isfield(h, 'baseH') && isfield(h.baseH, 'fig') && ishandle(h.baseH.fig)
            baseFig = h.baseH.fig;
        else
            baseFig = gcf;
        end
    end
    hb = guidata(baseFig);
    if isempty(h) || isempty(hb)
        warning('[BaseControl] 未能读取基座控制窗口的 guidata。');
        return;
    end
    if ~isfield(hb, 'sliders') || numel(hb.sliders) < 6
        warning('[BaseControl] 控制窗口数据不完整，缺少 sliders 字段。');
        return;
    end

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
    if ~any(strcmp(h.robot.BodyNames, 'ur10'))
        error('机器人模型中缺少物理基座 link "ur10"，当前 BodyNames: %s', strjoin(h.robot.BodyNames, ', '));
    end
    baseBody = getBody(h.robot, 'ur10');
    setFixedTransform(baseBody.Joint, tformBase);

    h.basePoseWorld = tformBase;
    h = rebuildIKSolver(h);
    guidata(mainFig, h);

    eeXYZ = tform2trvec(getTransform(h.robot, h.currentConfig, h.endEffector));
    fprintf('[BaseControl] 基座已更新: xyz=[%.3f, %.3f, %.3f], rpy(deg)=[%.1f, %.1f, %.1f]\n', ...
        bx, by, bz, rad2deg(br), rad2deg(bp), rad2deg(byaw));
    fprintf('[BaseControl] 当前末端世界坐标: [%.3f, %.3f, %.3f]\n', eeXYZ);

    if ~isfield(h, 'ax') || ~ishandle(h.ax) || ~strcmp(get(h.ax, 'Type'), 'axes')
        warning('主坐标轴句柄失效，跳过重绘。');
        return;
    end

    show(h.robot, h.currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
    setFixedWorkspace(h.ax);
    [startPosNow, endPosNow] = getPathPoints(h);
    ensureStartEndMarkers(h, startPosNow, endPosNow);

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
% 路径细分
% =========================================================================
function densePath = densifyPath(pathPoints, stepLen)
    if size(pathPoints, 1) <= 1
        densePath = pathPoints;
        return;
    end

    densePath = pathPoints(1, :);
    for i = 1:size(pathPoints, 1)-1
        p1 = pathPoints(i, :);
        p2 = pathPoints(i+1, :);
        seg = p2 - p1;
        L = norm(seg);
        n = max(1, ceil(L / max(stepLen, 1e-4)));
        t = (1:n)' / n;
        pts = p1 + t .* seg;
        densePath = [densePath; pts(2:end,:)]; %#ok<AGROW>
    end
end

%% =========================================================================
% 固定 5x5x5 工作空间
% =========================================================================
function setFixedWorkspace(ax)
    xlim(ax, [-2.5, 2.5]);
    ylim(ax, [-2.5, 2.5]);
    zlim(ax, [-2.5, 2.5]);
end

%% =========================================================================
% 确保起点/终点标记可见（show 清图后重画）
% =========================================================================
function ensureStartEndMarkers(h, startPos, endPos)
    if isfield(h, 'hStart') && ishandle(h.hStart)
        set(h.hStart, 'XData', startPos(1), 'YData', startPos(2), 'ZData', startPos(3), ...
            'Visible', 'on', 'Marker', 'o', 'MarkerSize', 12, ...
            'MarkerEdgeColor', 'g', 'LineWidth', 3);
    else
        h.hStart = plot3(h.ax, startPos(1), startPos(2), startPos(3), ...
            'go', 'MarkerSize', 12, 'LineWidth', 3);
    end

    if isfield(h, 'hEnd') && ishandle(h.hEnd)
        set(h.hEnd, 'XData', endPos(1), 'YData', endPos(2), 'ZData', endPos(3), ...
            'Visible', 'on', 'Marker', 'o', 'MarkerSize', 12, ...
            'MarkerEdgeColor', 'r', 'LineWidth', 3);
    else
        h.hEnd = plot3(h.ax, endPos(1), endPos(2), endPos(3), ...
            'ro', 'MarkerSize', 12, 'LineWidth', 3);
    end
end
