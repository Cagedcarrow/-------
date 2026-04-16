%% UR10 Mechanical Arm Planner (Body-Collision Focus, Refactored)
clc; clear; close all;

addpath('gui', 'kinematics', 'planner');

[robot, ik] = initWorldRobot();
endEffector = 'shovel_tip';

handles = MoveShovelTip(robot, @(src, event) startSimulation(ancestor(src, 'figure')));
handles.opts = defaultControlOptions();
handles.robot = robot;
handles.ik = ik;
handles.endEffector = endEffector;
handles.currentConfig = homeConfiguration(robot);
handles.basePoseWorld = eye(4);
collisionContext = buildShovelCollisionContext(robot);
handles.collisionContext = collisionContext;
handles.collisionFn = @(robotObj, varargin) shovelBodyCollision( ...
    robotObj, collisionContext, varargin{:});

view(handles.ax, 135, 30);
grid(handles.ax, 'on');
axis(handles.ax, 'equal');
setFixedWorkspace(handles.ax);
camlight(handles.ax);
lighting(handles.ax, 'gouraud');
title(handles.ax, 'UR10 机械臂本体规划（结构化重构版）');

[startPos, endPos] = getPathPoints(handles);
handles = ensureStartEndMarkers(handles, startPos, endPos);

mainFigHandle = handles.fig;
handles.baseH = BaseControl( ...
    @(src, event) updateBasePose(mainFigHandle, src), ...
    handles.opts.baseInitValues, ...
    handles.opts.baseLimits);
guidata(handles.fig, handles);
updateBasePose(mainFigHandle, handles.baseH.fig);

fprintf('系统已就绪：倒挂机械臂 + 铲子STL精确碰撞检查 + 起点自动吸附 + RRT规划。\n');

function startSimulation(figHandle)
    h = guidata(figHandle);
    if isempty(h)
        return;
    end
    if isfield(h, 'isRunning') && h.isRunning
        fprintf('已有任务在执行，本次请求忽略。\n');
        return;
    end
    if ~isfield(h, 'collisionFn') || isempty(h.collisionFn)
        h.collisionContext = buildShovelCollisionContext(h.robot);
        h.collisionFn = @(robotObj, varargin) shovelBodyCollision( ...
            robotObj, h.collisionContext, varargin{:});
    end

    h.isRunning = true;
    h = rebuildIKSolver(h);
    guidata(figHandle, h);
    cleanupObj = onCleanup(@() releaseRunLock(figHandle)); %#ok<NASGU>

    [startPos, endPos] = getPathPoints(h);
    h = ensureStartEndMarkers(h, startPos, endPos);
    guidata(figHandle, h);

    eeNow = getTransform(h.robot, h.currentConfig, h.endEffector);
    sharedOrientation = rotm2tform(tform2rotm(eeNow));

    fprintf('\n=== 执行开始 ===\n');
    fprintf('流程: 自动吸附到起点 -> RRT规划 -> 执行轨迹\n');
    fprintf('起点: [%.3f, %.3f, %.3f]\n', startPos);
    fprintf('终点: [%.3f, %.3f, %.3f]\n', endPos);
    fprintf('基座位姿平移: [%.3f, %.3f, %.3f]\n', h.basePoseWorld(1:3, 4));
    title(h.ax, '起点对位搜索中...', 'Color', 'b');
    drawnow;

    [startConfig, snapInfo] = solveStartConfig( ...
        h.robot, h.ik, h.endEffector, startPos, sharedOrientation, ...
        h.currentConfig, h.opts, h.collisionFn);

    fprintf(['起点对位结果: 误差=%.4f m, 最大步长=%.2f deg, 姿态索引=%d, 种子索引=%d\n'], ...
        snapInfo.positionError, rad2deg(snapInfo.maxJointStep), ...
        snapInfo.bestOrientationIndex, snapInfo.bestSeedIndex);

    if ~snapInfo.success
        warning('起点对位失败: %s', snapInfo.message);
        title(h.ax, '起点对位失败', 'Color', 'r');
        return;
    end
    if isfield(snapInfo, 'acceptedByRelaxed') && snapInfo.acceptedByRelaxed
        warning('起点对位采用放宽阈值: %s', snapInfo.message);
    end
    fprintf('起点对位成功: 误差=%.4f m, 最大步长=%.2f deg\n', ...
        snapInfo.positionError, rad2deg(snapInfo.maxJointStep));

    % 按需求：点击后直接吸附到起点（不执行初始 MoveJ 过渡段）
    h.currentConfig = startConfig;
    show(h.robot, h.currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
    setFixedWorkspace(h.ax);
    h = ensureStartEndMarkers(h, startPos, endPos);
    startXYZ = getCurrentToolPosition(h.robot, h.currentConfig, h.endEffector);
    eeStart = getTransform(h.robot, h.currentConfig, h.endEffector);
    sharedOrientation = rotm2tform(tform2rotm(eeStart));

    if isfield(h, 'hTrail') && ishandle(h.hTrail)
        delete(h.hTrail);
    end
    h.hTrail = plot3(h.ax, startXYZ(1), startXYZ(2), startXYZ(3), 'b-', 'LineWidth', 2);
    trailData = startXYZ;
    guidata(figHandle, h);
    drawnow;

    fprintf('已自动吸附到起点: EE=[%.3f, %.3f, %.3f], err=%.4f m\n', ...
        startXYZ, norm(startXYZ - startPos));
    title(h.ax, '起点吸附完成，目标可达性探测中...', 'Color', 'b');

    [goalProbeOK, goalProbe] = probeGoalReachability( ...
        h.robot, h.ik, h.endEffector, endPos, h.currentConfig, h.opts, h.collisionFn);
    fprintf(['目标可达性探测: ok=%d, best_err=%.4f m, best_xyz=[%.3f, %.3f, %.3f], ' ...
        'goal=[%.3f, %.3f, %.3f]\n'], ...
        goalProbeOK, goalProbe.bestErr, goalProbe.bestXYZ, endPos);
    if ~goalProbeOK && goalProbe.bestErr > h.opts.goalProbeLooseError
        warning('目标点在当前基座/工具配置下不可达或近不可达，建议调整终点或基座。');
        title(h.ax, '目标点不可达（先调整终点/基座）', 'Color', 'r');
        return;
    end
    if ~goalProbeOK
        warning(['目标点严格可达性不足，但仍在放宽阈值内，将继续尝试RRT。' ...
            ' best_err=%.4f m, loose=%.4f m'], ...
            goalProbe.bestErr, h.opts.goalProbeLooseError);
        fprintf(['继续规划调试信息: best_cfg可用=%d, 当前末端=[%.3f, %.3f, %.3f], ' ...
            '目标=[%.3f, %.3f, %.3f]\n'], ...
            ~isempty(goalProbe.bestCfg), ...
            getCurrentToolPosition(h.robot, h.currentConfig, h.endEffector), endPos);
    end

    plannerParams = struct();
    plannerParams.maxIter = h.opts.rrtMaxIter;
    plannerParams.goalRadius = h.opts.rrtGoalRadius;
    plannerParams.enableEnvCheck = false;
    plannerParams.enableRobotCheck = true;
    plannerParams.collisionFn = h.collisionFn;
    plannerParams.targetOrientation = sharedOrientation;
    plannerParams.enableShortcut = false;
    plannerParams.robotCheckStep = h.opts.collisionStep;
    plannerParams.maxIKPosError = h.opts.rrtMaxIKPosError;
    plannerParams.maxIKPosErrorLoose = h.opts.rrtMaxIKPosErrorLoose;
    plannerParams.maxPlanTimeSec = h.opts.rrtMaxPlanTimeSec;
    plannerParams.progressEvery = h.opts.rrtProgressEvery;
    plannerParams.tryDirectFirst = true;
    plannerParams.directSamples = h.opts.rrtDirectSamples;
    plannerParams.orientationYawOffsetsDeg = h.opts.rrtYawOffsetsDeg;
    plannerParams.orientationPitchOffsetsDeg = h.opts.rrtPitchOffsetsDeg;
    plannerParams.orientationRollOffsetsDeg = h.opts.rrtRollOffsetsDeg;
    plannerParams.acceptLooseIK = h.opts.rrtAcceptLooseIK;
    plannerParams.forcePositionOnlyIK = true;
    plannerParams.ikCandidateTopK = h.opts.rrtIKTopK;

    fprintf('调用 RRT 规划起点到终点（优先可达性）...\n');
    plannerWeights = [1 1 1 0 0 0];
    [~, planInfo] = rrtPlanner( ...
        startPos, endPos, h.robot, h.ik, plannerWeights, h.currentConfig, [], plannerParams);

    if ~isfield(planInfo, 'isReached') || ~planInfo.isReached || ...
            ~isfield(planInfo, 'rawPathConfigs') || isempty(planInfo.rawPathConfigs)
        if isfield(planInfo, 'rejectStats')
            rs = planInfo.rejectStats;
            fprintf(['RRT拒绝统计: ik_failed=%d, ik_pos_error=%d, node_collision=%d, ' ...
                'motion_collision=%d, env_edge=%d\n'], ...
                rs.ik_failed, rs.ik_pos_error, rs.node_collision, rs.motion_collision, rs.env_edge);
        end
        fprintf(['RRT失败调试信息: maxIter=%d, maxPlanTime=%.1fs, goalRadius=%.3f, ' ...
            'acceptLooseIK=%d, IKLoose=%.3f\n'], ...
            plannerParams.maxIter, plannerParams.maxPlanTimeSec, ...
            plannerParams.goalRadius, plannerParams.acceptLooseIK, plannerParams.maxIKPosErrorLoose);
        warning('RRT 失败或未返回关节锚点，停止执行。');
        title(h.ax, 'RRT 规划失败', 'Color', 'r');
        return;
    end

    jointPath = planInfo.rawPathConfigs;
    fprintf('RRT 成功: 树节点=%d, 锚点=%d, 用时=%.3fs\n', ...
        size(planInfo.tree, 1), size(jointPath, 1), planInfo.time);

    if isfield(planInfo, 'rawPathPoints') && ~isempty(planInfo.rawPathPoints)
        [jointPathCalib, fitInfo] = calibrateJointPathToTargets( ...
            h.robot, h.ik, h.endEffector, planInfo.rawPathPoints, jointPath, h.opts, h.collisionFn);
        fprintf('RRT锚点校准: success=%d, max_err=%.4f m, fail_index=%d\n', ...
            fitInfo.success, fitInfo.maxErr, fitInfo.failIndex);
        if ~fitInfo.success
            warning('RRT锚点校准失败，回退原始锚点: %s', fitInfo.message);
            title(h.ax, 'RRT 锚点校准失败，回退原轨迹', 'Color', [0.85 0.45 0]);
        else
            jointPath = jointPathCalib;
        end
    end

    lastCfgBeforePlayback = jointPath(end, :);
    lastXYZBeforePlayback = getCurrentToolPosition(h.robot, lastCfgBeforePlayback, h.endEffector);
    lastErrBeforePlayback = norm(lastXYZBeforePlayback - endPos);
    fprintf('RRT末端锚点误差: err=%.4f m, xyz=[%.3f, %.3f, %.3f]\n', ...
        lastErrBeforePlayback, lastXYZBeforePlayback);
    if lastErrBeforePlayback > h.opts.maxFinalError
        [endCfgPlan, endInfoPlan] = solveEndConfig( ...
            h.robot, h.ik, h.endEffector, endPos, lastCfgBeforePlayback, h.opts, h.collisionFn);
        fprintf(['RRT末端锚定追加: success=%d, posErr=%.4f, maxStep=%.2fdeg, ' ...
            'oriIdx=%d, seedIdx=%d\n'], ...
            endInfoPlan.success, endInfoPlan.positionError, ...
            rad2deg(endInfoPlan.maxJointStep), endInfoPlan.bestOrientationIndex, ...
            endInfoPlan.bestSeedIndex);
        if endInfoPlan.success
            jointPath = [jointPath; endCfgPlan];
        else
            fprintf('RRT末端锚定追加失败: %s\n', endInfoPlan.message);
        end
    end

    playbackPath = densifyJointPath(jointPath, h.opts.playbackJointStep);
    [safePath, msgPath] = validateJointPath(h.robot, playbackPath, h.opts.collisionStep, h.collisionFn);
    if ~safePath
        warning('执行轨迹无效: %s', msgPath);
        title(h.ax, '执行轨迹碰撞', 'Color', 'r');
        return;
    end

    title(h.ax, '执行 RRT 轨迹', 'Color', 'b');
    [finalConfig, trailData, okPath] = executeJointTrajectory( ...
        figHandle, playbackPath, trailData, startPos, endPos, 'RRT');
    if ~okPath
        return;
    end

    h = guidata(figHandle);
    h.currentConfig = finalConfig;
    h = ensureStartEndMarkers(h, startPos, endPos);
    guidata(figHandle, h);

    finalXYZ = getCurrentToolPosition(h.robot, finalConfig, h.endEffector);
    finalErr = norm(finalXYZ - endPos);
    fprintf('末端验收: final=[%.3f, %.3f, %.3f], target=[%.3f, %.3f, %.3f], err=%.4f m\n', ...
        finalXYZ, endPos, finalErr);
    if finalErr > h.opts.maxFinalError
        fprintf('触发末端补偿: 初始终点误差=%.4f m\n', finalErr);
        [endCfg, endInfo] = solveEndConfig( ...
            h.robot, h.ik, h.endEffector, endPos, finalConfig, h.opts, h.collisionFn);
        if endInfo.success
            endPath = densifyJointPath([finalConfig; endCfg], h.opts.playbackJointStep);
            [safeEnd, msgEnd] = validateJointPath(h.robot, endPath, h.opts.collisionStep, h.collisionFn);
            if safeEnd
                [finalConfig2, trailData, okEnd] = executeJointTrajectory( ...
                    figHandle, endPath, trailData, startPos, endPos, 'EndFix');
                if okEnd
                    h = guidata(figHandle);
                    h.currentConfig = finalConfig2;
                    guidata(figHandle, h);
                    finalConfig = finalConfig2;
                    finalXYZ = getCurrentToolPosition(h.robot, finalConfig, h.endEffector);
                    finalErr = norm(finalXYZ - endPos);
                    fprintf('末端补偿后误差: %.4f m\n', finalErr);
                end
            else
                warning('末端补偿轨迹无效: %s', msgEnd);
            end
        else
            warning('末端补偿求解失败: %s', endInfo.message);
        end
    end

    if finalErr > h.opts.maxFinalError
        warning('执行结束但未到达终点，末端误差 %.4f m', finalErr);
        title(h.ax, '执行结束但未到达终点', 'Color', 'r');
        return;
    end

    title(h.ax, '自动起点吸附 + RRT 执行完成');
    fprintf('执行完成: 播放帧=%d, 轨迹点=%d\n\n', size(playbackPath, 1), size(trailData, 1));
end

function opts = defaultControlOptions()
    opts = struct();
    opts.baseInitValues = [0 0 1.40 180 0 0];
    opts.baseLimits = [-2 2; -2 2; 0 2; -180 180; -180 180; -180 180];
    opts.ikWeights = [1 1 1 0.10 0.10 0.10];
    opts.startIKWeights = [1 1 1 0.02 0.02 0.02];
    opts.moveJStep = deg2rad(2.0);
    opts.playbackJointStep = deg2rad(1.0);
    opts.collisionStep = deg2rad(1.5);
    opts.maxIKPosError = 0.035;
    opts.maxStartSnapErrorPreferred = 0.06;
    opts.maxStartSnapError = 0.20;
    opts.framePause = 0.02;
    opts.reportInterval = 20;
    opts.rrtMaxIter = 1600;
    opts.rrtGoalRadius = 0.05;
    opts.rrtProgressEvery = 30;
    opts.rrtMaxPlanTimeSec = 90;
    opts.rrtDirectSamples = 30;
    opts.rrtMaxIKPosError = 0.08;
    opts.rrtMaxIKPosErrorLoose = 0.16;
    opts.rrtAcceptLooseIK = true;
    opts.rrtIKTopK = 5;
    opts.rrtYawOffsetsDeg = [0 90 -90 180];
    opts.rrtPitchOffsetsDeg = [0 -15 15 -30 30];
    opts.rrtRollOffsetsDeg = [0 -15 15];
    opts.startYawOffsetsDeg = [0 90 -90 180];
    opts.startPitchOffsetsDeg = [0 -15 15 -30 30 -45 45 -60 60];
    opts.startRollOffsetsDeg = [0 -15 15];
    opts.startProgressEvery = 20;
    opts.startSearchTopK = 10;
    opts.startSearchMaxTimeSec = 8.0;
    opts.startPosGate = 0.35;
    opts.startCollisionStep = deg2rad(3.0);
    opts.startRequireTransitionCollision = false;
    opts.execIKWeights = [1 1 1 0 0 0];
    opts.maxExecPointError = 0.05;
    opts.maxExecPointErrorLoose = 0.20;
    opts.maxFinalError = 0.08;
    opts.endIKWeights = [1 1 1 0 0 0];
    opts.maxEndSnapError = 0.12;
    opts.maxEndSnapErrorLoose = 0.22;
    opts.endYawOffsetsDeg = [0 90 -90 180];
    opts.endPitchOffsetsDeg = [0 -15 15 -30 30];
    opts.endRollOffsetsDeg = [0 -15 15];
    opts.endSearchTopK = 12;
    opts.endProgressEvery = 25;
    opts.goalProbeWeights = [1 1 1 0 0 0];
    opts.goalProbeAcceptError = 0.08;
    opts.goalProbeLooseError = 0.35;
    opts.goalProbeProgressEvery = 20;
end

function [jointPathOut, info] = calibrateJointPathToTargets( ...
    robot, ik, endEffector, targetPoints, seedJointPath, opts, collisionFn)

    n = min(size(targetPoints, 1), size(seedJointPath, 1));
    if n < 2
        jointPathOut = seedJointPath;
        info = struct('success', true, 'message', 'short_path', 'failIndex', 0, 'maxErr', 0);
        return;
    end

    jointPathOut = zeros(n, size(seedJointPath, 2));
    jointPathOut(1, :) = seedJointPath(1, :);

    info = struct('success', true, 'message', 'ok', 'failIndex', 0, 'maxErr', 0);
    homeCfg = homeConfiguration(robot);

    for i = 2:n
        targetXYZ = targetPoints(i, :);
        targetTform = trvec2tform(targetXYZ);
        prevCfg = jointPathOut(i-1, :);

        seeds = [prevCfg; seedJointPath(i, :); homeCfg];
        bestCfg = [];
        bestErr = inf;
        bestScore = inf;
        looseCfg = [];
        looseErr = inf;
        looseScore = inf;

        for s = 1:size(seeds, 1)
            [cfgTry, ikOK] = solveIK(ik, endEffector, targetTform, opts.execIKWeights, seeds(s, :));
            if ~ikOK
                continue;
            end
            cfgTry = unwrapConfigNearReference(cfgTry, prevCfg);
            xyzTry = getCurrentToolPosition(robot, cfgTry, endEffector);
            posErr = norm(xyzTry - targetXYZ);
            if posErr > opts.maxExecPointErrorLoose
                continue;
            end

            [hitNode, ~] = collisionFn(robot, cfgTry);
            if hitNode
                continue;
            end
            [hitMotion, ~] = collisionFn(robot, prevCfg, cfgTry, opts.collisionStep);
            if hitMotion
                continue;
            end

            stepPenalty = max(abs(cfgTry - prevCfg));
            score = posErr + 0.02 * stepPenalty;
            if posErr <= opts.maxExecPointError && score < bestScore
                bestScore = score;
                bestCfg = cfgTry;
                bestErr = posErr;
            end
            if score < looseScore
                looseScore = score;
                looseCfg = cfgTry;
                looseErr = posErr;
            end
        end

        if isempty(bestCfg)
            if ~isempty(looseCfg)
                bestCfg = looseCfg;
                bestErr = looseErr;
            else
                info.success = false;
                info.failIndex = i;
                info.message = sprintf('路径点 %d 无法校准到目标', i);
                return;
            end
        end

        jointPathOut(i, :) = bestCfg;
        info.maxErr = max(info.maxErr, bestErr);
    end
end

function [endConfig, info] = solveEndConfig( ...
    robot, ik, endEffector, endPos, currentConfig, opts, collisionFn)

    endConfig = [];
    info = struct('success', false, 'message', '', ...
        'positionError', inf, 'maxJointStep', inf, ...
        'bestOrientationIndex', 0, 'bestSeedIndex', 0);

    seeds = [currentConfig; homeConfiguration(robot)];
    oriCandidates = orientationCandidates( ...
        eye(4), opts.endYawOffsetsDeg, opts.endPitchOffsetsDeg, opts.endRollOffsetsDeg);

    evalCount = 0;
    candidateCfg = [];
    candidateErr = [];
    candidateStep = [];
    candidateScore = [];
    candidateOriIdx = [];
    candidateSeedIdx = [];

    for oi = 1:numel(oriCandidates)
        targetTform = trvec2tform(endPos) * oriCandidates{oi};
        for s = 1:size(seeds, 1)
            evalCount = evalCount + 1;
            [cfgTry, ok] = solveIK(ik, endEffector, targetTform, opts.endIKWeights, seeds(s, :));
            if ~ok
                if mod(evalCount, opts.endProgressEvery) == 0
                    fprintf('末端补偿搜索进度: %d, 候选=%d\n', evalCount, size(candidateCfg, 1));
                    drawnow limitrate;
                end
                continue;
            end

            cfgTry = unwrapConfigNearReference(cfgTry, currentConfig);
            xyzTry = getCurrentToolPosition(robot, cfgTry, endEffector);
            posErr = norm(xyzTry - endPos);
            if posErr > opts.maxEndSnapErrorLoose
                if mod(evalCount, opts.endProgressEvery) == 0
                    fprintf('末端补偿搜索进度: %d, 候选=%d\n', evalCount, size(candidateCfg, 1));
                    drawnow limitrate;
                end
                continue;
            end

            [hitNode, ~] = collisionFn(robot, cfgTry);
            if hitNode
                if mod(evalCount, opts.endProgressEvery) == 0
                    fprintf('末端补偿搜索进度: %d, 候选=%d\n', evalCount, size(candidateCfg, 1));
                    drawnow limitrate;
                end
                continue;
            end

            [hitMotion, ~] = collisionFn(robot, currentConfig, cfgTry, opts.collisionStep);
            if hitMotion
                if mod(evalCount, opts.endProgressEvery) == 0
                    fprintf('末端补偿搜索进度: %d, 候选=%d\n', evalCount, size(candidateCfg, 1));
                    drawnow limitrate;
                end
                continue;
            end

            maxStep = max(abs(cfgTry - currentConfig));
            score = posErr + 0.02 * maxStep;
            candidateCfg = [candidateCfg; cfgTry]; %#ok<AGROW>
            candidateErr = [candidateErr; posErr]; %#ok<AGROW>
            candidateStep = [candidateStep; maxStep]; %#ok<AGROW>
            candidateScore = [candidateScore; score]; %#ok<AGROW>
            candidateOriIdx = [candidateOriIdx; oi]; %#ok<AGROW>
            candidateSeedIdx = [candidateSeedIdx; s]; %#ok<AGROW>

            if mod(evalCount, opts.endProgressEvery) == 0
                fprintf('末端补偿搜索进度: %d, 候选=%d\n', evalCount, size(candidateCfg, 1));
                drawnow limitrate;
            end
        end
    end

    if isempty(candidateCfg)
        info.message = '末端补偿未找到可用无碰撞IK解';
        return;
    end

    [~, order] = sort(candidateScore, 'ascend');
    topK = min(opts.endSearchTopK, numel(order));
    order = order(1:topK);

    pickIdx = 0;
    for i = 1:topK
        idx = order(i);
        if candidateErr(idx) <= opts.maxEndSnapError
            pickIdx = idx;
            break;
        end
    end
    if pickIdx == 0
        pickIdx = order(1);
        info.message = sprintf('末端补偿采用放宽阈值: err=%.4f m', candidateErr(pickIdx));
    else
        info.message = 'ok';
    end

    endConfig = candidateCfg(pickIdx, :);
    info.positionError = candidateErr(pickIdx);
    info.maxJointStep = candidateStep(pickIdx);
    info.bestOrientationIndex = candidateOriIdx(pickIdx);
    info.bestSeedIndex = candidateSeedIdx(pickIdx);
    info.success = true;
end

function [ok, probe] = probeGoalReachability( ...
    robot, ik, endEffector, goalPos, currentConfig, opts, collisionFn)

    probe = struct('bestErr', inf, 'bestXYZ', [NaN NaN NaN], 'bestCfg', []);
    ok = false;

    seeds = [currentConfig; homeConfiguration(robot)];
    yawSet = deg2rad([0 90 -90 180]);
    pitchSet = deg2rad([0 -20 20 -40 40]);
    rollSet = deg2rad([0 -20 20]);
    totalChecks = size(seeds, 1) * numel(yawSet) * numel(pitchSet) * numel(rollSet);
    checkCount = 0;

    for s = 1:size(seeds, 1)
        for iy = 1:numel(yawSet)
            for ip = 1:numel(pitchSet)
                for ir = 1:numel(rollSet)
                    checkCount = checkCount + 1;
                    tform = trvec2tform(goalPos) * eul2tform([yawSet(iy) pitchSet(ip) rollSet(ir)], 'ZYX');
                    [cfgTry, ikOK] = solveIK(ik, endEffector, tform, opts.goalProbeWeights, seeds(s, :));
                    if ~ikOK
                        if mod(checkCount, opts.goalProbeProgressEvery) == 0
                            fprintf('目标可达性探测进度: %d/%d, 当前best_err=%.4f\n', ...
                                checkCount, totalChecks, probe.bestErr);
                            drawnow limitrate;
                        end
                        continue;
                    end
                    cfgTry = unwrapConfigNearReference(cfgTry, currentConfig);
                    [hitNode, ~] = collisionFn(robot, cfgTry);
                    if hitNode
                        if mod(checkCount, opts.goalProbeProgressEvery) == 0
                            fprintf('目标可达性探测进度: %d/%d, 当前best_err=%.4f\n', ...
                                checkCount, totalChecks, probe.bestErr);
                            drawnow limitrate;
                        end
                        continue;
                    end
                    xyzTry = getCurrentToolPosition(robot, cfgTry, endEffector);
                    err = norm(xyzTry - goalPos);
                    if err < probe.bestErr
                        probe.bestErr = err;
                        probe.bestXYZ = xyzTry;
                        probe.bestCfg = cfgTry;
                    end
                    if mod(checkCount, opts.goalProbeProgressEvery) == 0
                        fprintf('目标可达性探测进度: %d/%d, 当前best_err=%.4f\n', ...
                            checkCount, totalChecks, probe.bestErr);
                        drawnow limitrate;
                    end
                end
            end
        end
    end

    if probe.bestErr <= opts.goalProbeAcceptError
        ok = true;
        return;
    end

    if probe.bestErr <= opts.goalProbeLooseError
        fprintf('目标点可达性偏弱: best_err=%.4f m（可放宽执行）\n', probe.bestErr);
    end
end

function [startConfig, info] = solveStartConfig( ...
    robot, ik, endEffector, startPos, baseOrientation, currentConfig, opts, collisionFn)

    startConfig = [];
    info = struct();
    info.success = false;
    info.message = '';
    info.positionError = inf;
    info.maxJointStep = inf;
    info.candidateCount = 0;
    info.ikSuccessCount = 0;
    info.ikFailCount = 0;
    info.collisionRejectCount = 0;
    info.bestOrientationIndex = 0;
    info.bestSeedIndex = 0;
    info.acceptedByRelaxed = false;
    info.timeout = false;
    info.elapsed = 0;

    seeds = {currentConfig, homeConfiguration(robot)};
    oriCandidates = orientationCandidates( ...
        baseOrientation, opts.startYawOffsetsDeg, ...
        opts.startPitchOffsetsDeg, opts.startRollOffsetsDeg);
    info.candidateCount = numel(oriCandidates);
    evalCount = 0;
    t0 = tic;

    cfgList = [];
    scoreList = [];
    posErrList = [];
    maxStepList = [];
    oriIdxList = [];
    seedIdxList = [];

    for oi = 1:numel(oriCandidates)
        if toc(t0) > opts.startSearchMaxTimeSec
            info.timeout = true;
            break;
        end
        targetTform = trvec2tform(startPos) * oriCandidates{oi};
        for si = 1:numel(seeds)
            if toc(t0) > opts.startSearchMaxTimeSec
                info.timeout = true;
                break;
            end

            evalCount = evalCount + 1;
            [cfgTry, ok] = solveIK(ik, endEffector, targetTform, opts.startIKWeights, seeds{si});
            if ~ok
                info.ikFailCount = info.ikFailCount + 1;
                if mod(evalCount, opts.startProgressEvery) == 0
                    fprintf('起点搜索进度: %d/%d, IK成功=%d, IK失败=%d\n', ...
                        evalCount, info.candidateCount * numel(seeds), ...
                        info.ikSuccessCount, info.ikFailCount);
                    drawnow limitrate;
                end
                continue;
            end
            info.ikSuccessCount = info.ikSuccessCount + 1;

            cfgTry = unwrapConfigNearReference(cfgTry, currentConfig);
            posErr = norm(startPos - tform2trvec(getTransform(robot, cfgTry, endEffector)));
            if posErr > opts.startPosGate
                if mod(evalCount, opts.startProgressEvery) == 0
                    fprintf('起点搜索进度: %d/%d, IK成功=%d, IK失败=%d\n', ...
                        evalCount, info.candidateCount * numel(seeds), ...
                        info.ikSuccessCount, info.ikFailCount);
                    drawnow limitrate;
                end
                continue;
            end

            maxStep = max(abs(cfgTry - currentConfig));
            score = posErr + 0.03 * maxStep;
            cfgList = [cfgList; cfgTry]; %#ok<AGROW>
            scoreList = [scoreList; score]; %#ok<AGROW>
            posErrList = [posErrList; posErr]; %#ok<AGROW>
            maxStepList = [maxStepList; maxStep]; %#ok<AGROW>
            oriIdxList = [oriIdxList; oi]; %#ok<AGROW>
            seedIdxList = [seedIdxList; si]; %#ok<AGROW>

            if mod(evalCount, opts.startProgressEvery) == 0
                fprintf('起点搜索进度: %d/%d, IK成功=%d, IK失败=%d, 候选=%d\n', ...
                    evalCount, info.candidateCount * numel(seeds), ...
                    info.ikSuccessCount, info.ikFailCount, numel(scoreList));
                drawnow limitrate;
            end
        end
        if info.timeout
            break;
        end
    end

    info.elapsed = toc(t0);

    if isempty(scoreList)
        fprintf('起点对位统计: 姿态候选=%d, IK成功=%d, IK失败=%d, 候选=0, 用时=%.2fs\n', ...
            info.candidateCount, info.ikSuccessCount, info.ikFailCount, info.elapsed);
        if info.timeout
            info.message = sprintf('起点搜索超时 %.2fs，未找到可用候选', info.elapsed);
        else
            info.message = sprintf('没有可用起点IK候选 (IK成功=%d)', info.ikSuccessCount);
        end
        return;
    end

    [~, order] = sort(scoreList, 'ascend');
    topK = min(opts.startSearchTopK, numel(order));
    order = order(1:topK);

    for k = 1:topK
        idx = order(k);
        cfgTry = cfgList(idx, :);

        [hitNode, ~] = collisionFn(robot, cfgTry);
        if hitNode
            info.collisionRejectCount = info.collisionRejectCount + 1;
            continue;
        end

        if opts.startRequireTransitionCollision
            [hitMotion, ~] = collisionFn(robot, currentConfig, cfgTry, opts.startCollisionStep);
            if hitMotion
                info.collisionRejectCount = info.collisionRejectCount + 1;
                continue;
            end
        end

        startConfig = cfgTry;
        info.positionError = posErrList(idx);
        info.maxJointStep = maxStepList(idx);
        info.bestOrientationIndex = oriIdxList(idx);
        info.bestSeedIndex = seedIdxList(idx);
        break;
    end

    fprintf('起点对位统计: 姿态候选=%d, IK成功=%d, IK失败=%d, 预选=%d, 碰撞剔除=%d, 用时=%.2fs\n', ...
        info.candidateCount, info.ikSuccessCount, info.ikFailCount, ...
        numel(scoreList), info.collisionRejectCount, info.elapsed);

    if isempty(startConfig)
        if info.timeout
            info.message = sprintf('起点搜索超时 %.2fs，候选均碰撞/不可达', info.elapsed);
        else
            info.message = sprintf('没有找到无碰撞起点IK解 (候选=%d, 碰撞剔除=%d)', ...
                numel(scoreList), info.collisionRejectCount);
        end
        return;
    end

    if info.positionError <= opts.maxStartSnapErrorPreferred
        info.success = true;
        info.message = 'ok';
        return;
    end

    if info.positionError <= opts.maxStartSnapError
        info.success = true;
        info.acceptedByRelaxed = true;
        info.message = sprintf(['起点误差 %.4f m 超过推荐阈值 %.3f m，' ...
            '采用放宽阈值 %.3f m'], ...
            info.positionError, opts.maxStartSnapErrorPreferred, opts.maxStartSnapError);
        return;
    end

    info.message = sprintf('起点误差过大 %.4f m (推荐<=%.3f, 放宽<=%.3f)', ...
        info.positionError, opts.maxStartSnapErrorPreferred, opts.maxStartSnapError);
end

function candidates = orientationCandidates(baseOrientation, yawOffsetsDeg, pitchOffsetsDeg, rollOffsetsDeg)
    baseEul = tform2eul(baseOrientation, 'ZYX');
    yawOffsets = deg2rad(yawOffsetsDeg(:));
    pitchOffsets = deg2rad(pitchOffsetsDeg(:));
    rollOffsets = deg2rad(rollOffsetsDeg(:));

    candidates = cell(0, 1);
    for iy = 1:numel(yawOffsets)
        for ip = 1:numel(pitchOffsets)
            for ir = 1:numel(rollOffsets)
                eul = [ ...
                    wrapToPiLocal(baseEul(1) + yawOffsets(iy)), ...
                    wrapToPiLocal(baseEul(2) + pitchOffsets(ip)), ...
                    wrapToPiLocal(baseEul(3) + rollOffsets(ir))];
                candidates{end+1, 1} = eul2tform(eul, 'ZYX'); %#ok<AGROW>
            end
        end
    end
end

function [isSafe, message] = validateJointPath(robot, jointPath, collisionStep, collisionFn)
    isSafe = true;
    message = 'ok';
    if isempty(jointPath)
        isSafe = false;
        message = '空路径';
        return;
    end

    [hitStart, hitInfo] = collisionFn(robot, jointPath(1, :));
    if hitStart
        isSafe = false;
        message = sprintf('起点碰撞: %s vs %s', hitInfo.body_a, hitInfo.body_b);
        return;
    end

    for i = 2:size(jointPath, 1)
        [hit, info] = collisionFn(robot, jointPath(i-1, :), jointPath(i, :), collisionStep);
        if hit
            isSafe = false;
            message = sprintf('段 %d 碰撞: %s vs %s (sample=%d)', ...
                i-1, info.body_a, info.body_b, info.sample_index);
            return;
        end
    end
end

function [finalConfig, trailData, success] = executeJointTrajectory( ...
    figHandle, jointPath, trailData, startPos, endPos, stageName)

    h = guidata(figHandle);
    finalConfig = h.currentConfig;
    success = true;

    for i = 1:size(jointPath, 1)
        h = guidata(figHandle);
        if isempty(h) || ~isfield(h, 'ax') || ~ishandle(h.ax)
            warning('坐标轴句柄失效，停止执行。');
            success = false;
            return;
        end

        if i >= 2
            if isfield(h, 'collisionFn') && ~isempty(h.collisionFn)
                [hit, hitInfo] = h.collisionFn( ...
                    h.robot, jointPath(i-1, :), jointPath(i, :), h.opts.collisionStep);
            else
                [hit, hitInfo] = robotCollision( ...
                    h.robot, jointPath(i-1, :), jointPath(i, :), h.opts.collisionStep);
            end
            if hit
                warning('%s 执行中碰撞: %s vs %s (sample=%d)', ...
                    stageName, hitInfo.body_a, hitInfo.body_b, hitInfo.sample_index);
                title(h.ax, [stageName ' 执行中碰撞停止'], 'Color', 'r');
                success = false;
                return;
            end
        end

        finalConfig = jointPath(i, :);
        actualXYZ = getCurrentToolPosition(h.robot, finalConfig, h.endEffector);

        show(h.robot, finalConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
        setFixedWorkspace(h.ax);
        h = ensureStartEndMarkers(h, startPos, endPos);

        trailData = [trailData; actualXYZ]; %#ok<AGROW>
        if ~isfield(h, 'hTrail') || ~ishandle(h.hTrail)
            h.hTrail = plot3(h.ax, trailData(:,1), trailData(:,2), trailData(:,3), ...
                'b-', 'LineWidth', 2);
        else
            set(h.hTrail, 'XData', trailData(:,1), 'YData', trailData(:,2), 'ZData', trailData(:,3));
        end

        if i <= 5 || mod(i, h.opts.reportInterval) == 0 || i == size(jointPath, 1)
            fprintf('%s 帧 %d/%d: EE=[%.3f, %.3f, %.3f]\n', ...
                stageName, i, size(jointPath, 1), actualXYZ);
        end

        guidata(figHandle, h);
        drawnow;
        pause(h.opts.framePause);
    end
end

function h = ensureStartEndMarkers(h, startPos, endPos)
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

function [startPos, endPos] = getPathPoints(h)
    if isfield(h, 'startSliders') && isfield(h, 'endSliders')
        startPos = zeros(1, 3);
        endPos = zeros(1, 3);
        for i = 1:3
            if ishandle(h.startSliders{i})
                startPos(i) = h.startSliders{i}.Value;
            else
                startPos(i) = 0.5;
            end
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

function updateBasePose(mainFig, src)
    if ~ishandle(mainFig)
        return;
    end

    h = guidata(mainFig);
    if isempty(h)
        return;
    end

    baseFig = [];
    if nargin >= 2 && ~isempty(src) && ishandle(src)
        baseFig = ancestor(src, 'figure');
    end
    if isempty(baseFig) || ~ishandle(baseFig)
        if isfield(h, 'baseH') && isfield(h.baseH, 'fig') && ishandle(h.baseH.fig)
            baseFig = h.baseH.fig;
        end
    end
    if isempty(baseFig) || ~ishandle(baseFig)
        return;
    end

    hb = guidata(baseFig);
    if isempty(hb) || ~isfield(hb, 'sliders') || numel(hb.sliders) < 6
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
    if any(strcmp(h.robot.BodyNames, 'ur10'))
        baseBody = getBody(h.robot, 'ur10');
    else
        baseBody = h.robot.Bodies{1};
    end
    setFixedTransform(baseBody.Joint, tformBase);

    h.basePoseWorld = tformBase;
    h = rebuildIKSolver(h);

    show(h.robot, h.currentConfig, 'Parent', h.ax, 'PreservePlot', false, 'FastUpdate', true);
    setFixedWorkspace(h.ax);
    [startPos, endPos] = getPathPoints(h);
    h = ensureStartEndMarkers(h, startPos, endPos);
    if isfield(h, 'hTrail') && ishandle(h.hTrail)
        set(h.hTrail, 'Visible', 'off');
    end
    guidata(mainFig, h);
    drawnow limitrate;

    eeXYZ = getCurrentToolPosition(h.robot, h.currentConfig, h.endEffector);
    fprintf('[BaseControl] xyz=[%.3f, %.3f, %.3f], rpy=[%.1f, %.1f, %.1f] deg, EE=[%.3f, %.3f, %.3f]\n', ...
        bx, by, bz, rad2deg(br), rad2deg(bp), rad2deg(byaw), eeXYZ);
end

function denseJointPath = densifyJointPath(jointPath, maxJointStep)
    if size(jointPath, 1) <= 1
        denseJointPath = jointPath;
        return;
    end

    denseJointPath = jointPath(1, :);
    for i = 1:size(jointPath, 1) - 1
        q1 = jointPath(i, :);
        q2 = jointPath(i+1, :);
        seg = q2 - q1;
        n = max(1, ceil(max(abs(seg)) / max(maxJointStep, 1e-6)));
        a = (1:n)' / n;
        pts = q1 + a .* seg;
        denseJointPath = [denseJointPath; pts]; %#ok<AGROW>
    end
end

function setFixedWorkspace(ax)
    xlim(ax, [-2.5, 2.5]);
    ylim(ax, [-2.5, 2.5]);
    zlim(ax, [-2.5, 2.5]);
end

function [robot, ik] = initWorldRobot()
    urdfFile = 'ur10_world.urdf';
    if ~exist(urdfFile, 'file')
        error('找不到文件: %s', urdfFile);
    end
    robot = importrobot(urdfFile);
    robot.DataFormat = 'row';
    ik = inverseKinematics('RigidBodyTree', robot);
    fprintf('--> 已加载机器人: %s, 刚体数=%d\n', urdfFile, robot.NumBodies);
end

function h = rebuildIKSolver(h)
    h.ik = inverseKinematics('RigidBodyTree', h.robot);
end

function xyz = getCurrentToolPosition(robot, config, endEffector)
    xyz = tform2trvec(getTransform(robot, config, endEffector));
end

function cfgOut = unwrapConfigNearReference(cfgIn, refCfg)
    cfgOut = cfgIn;
    for i = 1:numel(cfgIn)
        delta = cfgIn(i) - refCfg(i);
        delta = mod(delta + pi, 2*pi) - pi;
        cfgOut(i) = refCfg(i) + delta;
    end
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end

function ctx = buildShovelCollisionContext(robot)
    bodyNames = string(robot.BodyNames(:));
    shovelIdx = find(bodyNames == "铲子", 1);
    if isempty(shovelIdx)
        shovelIdx = find(contains(lower(bodyNames), "shovel") & bodyNames ~= "shovel_tip", 1);
    end
    if isempty(shovelIdx)
        error('未在机器人模型中找到铲子刚体（名称应为"铲子"或包含"shovel"）。');
    end

    shovelName = bodyNames(shovelIdx);
    excludeNames = ["shovel_tip", "ur10_wrist_3", "wrist_3"];

    try
        shovelBody = getBody(robot, char(shovelName));
        parentName = string(shovelBody.Parent);
        if strlength(parentName) > 0
            excludeNames = [excludeNames, parentName]; %#ok<AGROW>
        end
    catch
    end

    targetNames = setdiff(bodyNames, [shovelName; excludeNames(:)], 'stable');
    if isempty(targetNames)
        error('铲子碰撞检测目标为空，请检查URDF中的刚体命名。');
    end

    ctx = struct();
    ctx.shovelName = shovelName;
    ctx.targetNames = targetNames(:);
end

function [isColliding, info] = shovelBodyCollision(robot, ctx, configA, varargin)
    if nargin < 4
        [isColliding, info] = checkShovelConfigCollision(robot, ctx, configA);
        return;
    end

    configB = varargin{1};
    maxStepRad = deg2rad(2.0);
    if numel(varargin) >= 2 && ~isempty(varargin{2})
        maxStepRad = varargin{2};
    end

    delta = configB - configA;
    n = max(2, ceil(max(abs(delta)) / max(maxStepRad, 1e-6)) + 1);

    isColliding = false;
    info = defaultShovelCollisionInfo(ctx);
    info.mode = 'motion';

    for k = 1:n
        alpha = (k - 1) / max(n - 1, 1);
        cfg = configA + alpha * delta;
        [hitNow, cfgInfo] = checkShovelConfigCollision(robot, ctx, cfg);

        if cfgInfo.min_distance < info.min_distance
            info.min_distance = cfgInfo.min_distance;
            info.body_b = cfgInfo.body_b;
        end

        if hitNow
            isColliding = true;
            info = cfgInfo;
            info.mode = 'motion';
            info.sample_index = k;
            info.method = 'swept_shovel_stl_vs_body';
            return;
        end
    end
end

function [isColliding, info] = checkShovelConfigCollision(robot, ctx, config)
    [~, details] = checkCollision(robot, config, ...
        'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');

    labels = getCollisionBodyLabelsLocal(robot, details);
    info = defaultShovelCollisionInfo(ctx);

    shovelIdx = find(labels == ctx.shovelName, 1);
    if isempty(shovelIdx)
        info.method = 'shovel_label_missing';
        return;
    end

    isColliding = false;
    for i = 1:numel(ctx.targetNames)
        bodyName = ctx.targetNames(i);
        bodyIdx = find(labels == bodyName, 1);
        if isempty(bodyIdx)
            continue;
        end

        pairValue = details(shovelIdx, bodyIdx);
        reverseValue = details(bodyIdx, shovelIdx);
        pairDist = minFinitePairDistance(pairValue, reverseValue);
        if pairDist < info.min_distance
            info.min_distance = pairDist;
            info.body_b = char(bodyName);
        end

        if isCollisionEntryLocal(pairValue) || isCollisionEntryLocal(reverseValue)
            isColliding = true;
            info.body_b = char(bodyName);
            info.method = 'shovel_stl_vs_body';
            if ~isfinite(info.min_distance)
                info.min_distance = 0;
            end
            return;
        end
    end
end

function info = defaultShovelCollisionInfo(ctx)
    info = struct();
    info.mode = 'config';
    info.method = 'shovel_stl_vs_body';
    info.body_a = char(ctx.shovelName);
    info.body_b = '';
    info.min_distance = inf;
    info.sample_index = 0;
end

function labels = getCollisionBodyLabelsLocal(robot, details)
    baseLabel = "base";
    if isprop(robot, 'BaseName') && ~isempty(robot.BaseName)
        baseLabel = string(robot.BaseName);
    end

    labels = [baseLabel, string(robot.BodyNames)];
    if nargin >= 2 && size(details, 1) ~= numel(labels)
        labels = string(robot.BodyNames);
    end
end

function tf = isCollisionEntryLocal(value)
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

function dist = minFinitePairDistance(a, b)
    dist = inf;
    vals = [a, b];
    for i = 1:numel(vals)
        v = vals(i);
        if isfinite(v) && ~isnan(v)
            dist = min(dist, v);
        end
    end
end

function releaseRunLock(figHandle)
    if ~ishandle(figHandle)
        return;
    end
    h = guidata(figHandle);
    if isempty(h)
        return;
    end
    h.isRunning = false;
    guidata(figHandle, h);
end
