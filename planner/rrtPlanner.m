function [pathPoints, planInfo] = rrtPlanner(startPos, endPos, robot, ik, weights, currentConfig, varargin)
%RRTPLANNER DP-RRT (3D) with STL point-cloud obstacle avoidance.
% Coordinate convention:
%   startPos/endPos/obstaclePoints are all in WORLD coordinates.
% Backward compatible:
%   pathPoints = rrtPlanner(startPos, endPos, robot, ik, weights, currentConfig)
% New usage:
%   [pathPoints, info] = rrtPlanner(..., obstaclePoints)
%   [pathPoints, info] = rrtPlanner(..., obstaclePoints, userParams)

    % -------------------- input --------------------
    startPos = reshape(startPos, 1, 3);
    endPos = reshape(endPos, 1, 3);
    assert(numel(startPos) == 3 && numel(endPos) == 3, ...
        'rrtPlanner: startPos/endPos must be 1x3 world coordinates.');

    obstaclePoints = [];
    userParams = struct();

    if ~isempty(varargin)
        obstaclePoints = varargin{1};
        if ~isempty(obstaclePoints)
            assert(size(obstaclePoints,2) == 3, ...
                'rrtPlanner: obstaclePoints must be Nx3 world coordinates.');
        end
    end
    if numel(varargin) >= 2
        userParams = varargin{2};
    end

    % -------------------- params --------------------
    p = defaultParams(startPos, endPos, obstaclePoints);
    p = mergeStruct(p, userParams);

    if isempty(weights)
        weights = [0.1 0.1 0.1 1 1 1];
    end

    % -------------------- init --------------------
    treeXYZ = startPos;       % N x 3
    parent = 0;               % N x 1
    cost   = 0;               % N x 1
    qCfg   = currentConfig;   % N x DOF

    failCount = 0;
    reached = false;
    goalIdx = -1;
    tStart = tic;

    % -------------------- main loop --------------------
    for iter = 1:p.maxIter
        if mod(iter, 250) == 1
            fprintf('--> [DP-RRT] 迭代中: %d/%d, 当前树节点=%d\n', iter, p.maxIter, size(treeXYZ, 1));
        end

        PgCurrent  = p.PgInit * exp(-p.failDecay * failCount);
        rhoCurrent = p.rhoInit * exp(-p.failDecay * failCount);

        if rand < PgCurrent
            qRand = endPos;
        else
            qRand = [
                p.limits(1) + rand * (p.limits(2) - p.limits(1)), ...
                p.limits(3) + rand * (p.limits(4) - p.limits(3)), ...
                p.limits(5) + rand * (p.limits(6) - p.limits(5))
            ];
        end

        [nearIdx, qNear] = nearestNode(treeXYZ, qRand);

        vRand = unitVec(qRand - qNear);
        vGoal = unitVec(endPos - qNear);

        if norm(qRand - endPos) > 1e-9
            vNew = (1 - rhoCurrent) * vRand + rhoCurrent * vGoal;
            vNew = unitVec(vNew);
        else
            vNew = vGoal;
        end

        dMin = minDistPointToCloud(qNear, obstaclePoints);
        lambdaCurrent = adaptiveStep(dMin, p);

        qNew = qNear + lambdaCurrent * vNew;

        if ~inBounds(qNew, p.limits)
            failCount = failCount + 1;
            continue;
        end

        [isValid, newCfg] = isNodeAndEdgeValid( ...
            qNear, qNew, nearIdx, treeXYZ, qCfg, robot, ik, weights, p, obstaclePoints);

        if ~isValid
            failCount = failCount + 1;
            continue;
        end

        failCount = max(0, failCount - p.failRecover);

        treeXYZ = [treeXYZ; qNew]; %#ok<AGROW>
        parent  = [parent; nearIdx]; %#ok<AGROW>
        cost    = [cost; cost(nearIdx) + norm(qNew - qNear)]; %#ok<AGROW>
        qCfg    = [qCfg; newCfg]; %#ok<AGROW>

        if norm(qNew - endPos) <= p.goalRadius
            reached = true;
            goalIdx = size(treeXYZ, 1);
            break;
        end
    end

    % -------------------- extract --------------------
    if reached
        pathPoints = backtrackPath(treeXYZ, parent, goalIdx);
        if p.enableShortcut && size(pathPoints, 1) > 2
            pathPoints = shortcutPath(pathPoints, obstaclePoints, p);
        end
        fprintf('--> [DP-RRT] 路径规划成功！节点数=%d, 路径点=%d, 用时=%.3fs\n', ...
            size(treeXYZ,1), size(pathPoints,1), toc(tStart));
    else
        pathPoints = [];
        fprintf('--> [DP-RRT] 路径规划失败。迭代=%d, 用时=%.3fs\n', p.maxIter, toc(tStart));
    end

    if nargout > 1
        planInfo = struct();
        planInfo.isReached = reached;
        planInfo.tree = treeXYZ;
        planInfo.parent = parent;
        planInfo.cost = cost;
        planInfo.iter = iter;
        planInfo.time = toc(tStart);
        if reached
            planInfo.pathLength = pathLength(pathPoints);
        else
            planInfo.pathLength = inf;
        end
    end
end

% ==================== helpers ====================

function p = defaultParams(startPos, endPos, obstaclePoints)
    if isempty(obstaclePoints)
        minXYZ = min([startPos; endPos], [], 1) - 0.2;
        maxXYZ = max([startPos; endPos], [], 1) + 0.2;
    else
        obsMin = min(obstaclePoints, [], 1);
        obsMax = max(obstaclePoints, [], 1);
        minXYZ = min([startPos; endPos; obsMin], [], 1) - 0.1;
        maxXYZ = max([startPos; endPos; obsMax], [], 1) + 0.1;
    end

    p = struct();
    p.maxIter       = 2500;
    p.goalRadius    = 0.05;
    p.limits        = [minXYZ(1), maxXYZ(1), minXYZ(2), maxXYZ(2), minXYZ(3), maxXYZ(3)];
    p.lambdaMax     = 0.10;
    p.lambdaMin     = 0.015;
    p.dSafe         = 0.10;
    p.kappa         = 8.0;
    p.rhoInit       = 0.30;
    p.PgInit        = 0.25;
    p.failDecay     = 0.50;
    p.failRecover   = 2;
    p.envSafeDist   = 0.020;
    p.edgeSampleStep= 0.015;
    p.enableRobotCheck = true;
    p.enableEnvCheck   = true;
    p.targetOrientation = eul2tform([0 0 0], 'ZYX');
    p.enableShortcut = true;
    p.shortcutTrials = 80;
end

function out = mergeStruct(base, ext)
    out = base;
    if isempty(ext), return; end
    f = fieldnames(ext);
    for i = 1:numel(f)
        out.(f{i}) = ext.(f{i});
    end
end

function [idx, qNear] = nearestNode(treeXYZ, qRand)
    d = vecnorm(treeXYZ - qRand, 2, 2);
    [~, idx] = min(d);
    qNear = treeXYZ(idx, :);
end

function v = unitVec(v)
    n = norm(v);
    if n > 1e-12
        v = v / n;
    else
        v = [0 0 0];
    end
end

function tf = inBounds(q, lim)
    tf = q(1)>=lim(1) && q(1)<=lim(2) && ...
         q(2)>=lim(3) && q(2)<=lim(4) && ...
         q(3)>=lim(5) && q(3)<=lim(6);
end

function dMin = minDistPointToCloud(pt, obstaclePoints)
    if isempty(obstaclePoints)
        dMin = inf;
        return;
    end
    dMin = min(vecnorm(obstaclePoints - pt, 2, 2));
end

function lambda = adaptiveStep(dMin, p)
    if ~isfinite(dMin)
        lambda = p.lambdaMax;
        return;
    end
    if dMin >= p.dSafe
        lambda = p.lambdaMax;
        return;
    end
    num = exp(p.kappa * dMin) - 1;
    den = exp(p.kappa * p.dSafe) - 1;
    if den <= 1e-12
        alpha = 0;
    else
        alpha = max(0, min(1, num / den));
    end
    lambda = p.lambdaMin + (p.lambdaMax - p.lambdaMin) * alpha;
end

function [isValid, cfgOut] = isNodeAndEdgeValid( ...
    qNear, qNew, nearIdx, treeXYZ, qCfg, robot, ik, weights, p, obstaclePoints)

    isValid = true;
    cfgOut = qCfg(nearIdx, :);

    % 1) 环境边碰撞：用线段采样点到点云的最小距离近似
    if p.enableEnvCheck && ~isempty(obstaclePoints)
        if edgeNearObstacle(qNear, qNew, obstaclePoints, p.envSafeDist, p.edgeSampleStep)
            isValid = false;
            return;
        end
    end

    % 2) 机器人可达 + 自碰撞
    if p.enableRobotCheck
        targetTform = trvec2tform(qNew) * p.targetOrientation;
        [cfgCandidate, ok] = solveIK(ik, 'shovel_tip', targetTform, weights, qCfg(nearIdx,:));
        if ~ok
            isValid = false;
            return;
        end

        % 检查当前候选点以及从 nearCfg 到 cfgCandidate 的关节运动过程
        isSelfColliding = isShovelBodyMotionCollision(robot, qCfg(nearIdx,:), cfgCandidate);

        if isSelfColliding
            isValid = false;
            return;
        end
        cfgOut = cfgCandidate;
    end
end

function tf = edgeNearObstacle(q1, q2, obstaclePoints, safeDist, stepLen)
    seg = q2 - q1;
    L = norm(seg);
    if L < 1e-9
        tf = minDistPointToCloud(q1, obstaclePoints) < safeDist;
        return;
    end

    n = max(2, ceil(L / max(stepLen, 1e-4)) + 1);
    t = linspace(0, 1, n)';
    samples = q1 + t .* seg;
    tf = false;

    for i = 1:size(samples,1)
        if minDistPointToCloud(samples(i,:), obstaclePoints) < safeDist
            tf = true;
            return;
        end
    end
end

function path = backtrackPath(treeXYZ, parent, goalIdx)
    idx = goalIdx;
    path = treeXYZ(idx, :);
    while idx > 1
        idx = parent(idx);
        path = [treeXYZ(idx, :); path]; %#ok<AGROW>
    end
end

function path = shortcutPath(path, obstaclePoints, p)
    if size(path,1) <= 2 || isempty(obstaclePoints)
        return;
    end

    for k = 1:p.shortcutTrials
        if size(path,1) <= 2, break; end
        i = randi([1, size(path,1)-1]);
        j = randi([i+1, size(path,1)]);
        if j <= i+1
            continue;
        end
        if ~edgeNearObstacle(path(i,:), path(j,:), obstaclePoints, p.envSafeDist, p.edgeSampleStep)
            path = [path(1:i,:); path(j:end,:)];
        end
    end
end

function L = pathLength(path)
    if size(path,1) < 2
        L = 0;
    else
        L = sum(vecnorm(diff(path,1,1), 2, 2));
    end
end

function tf = isShovelBodyCollision(robot, config)
    tf = false;

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

        if isCollisionEntry(details(shovelIdx, bodyIdx)) || ...
           isCollisionEntry(details(bodyIdx, shovelIdx))
            tf = true;
            return;
        end
    end
end

function tf = isShovelBodyMotionCollision(robot, configA, configB)
    tf = false;
    delta = configB - configA;
    maxJointStep = deg2rad(2.0);
    numSamples = max(2, ceil(max(abs(delta)) / maxJointStep) + 1);

    for k = 1:numSamples
        alpha = (k - 1) / max(numSamples - 1, 1);
        cfg = configA + alpha * delta;
        if isShovelBodyCollision(robot, cfg)
            tf = true;
            return;
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
