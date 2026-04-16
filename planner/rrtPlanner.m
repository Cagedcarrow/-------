function [pathPoints, planInfo] = rrtPlanner(startPos, endPos, robot, ik, weights, currentConfig, varargin)
%RRTPLANNER Fast DP-RRT with robot self-collision checks.
% Backward compatible:
%   pathPoints = rrtPlanner(startPos, endPos, robot, ik, weights, currentConfig)
%   [pathPoints, info] = rrtPlanner(..., obstaclePoints)
%   [pathPoints, info] = rrtPlanner(..., obstaclePoints, userParams)

    startPos = reshape(startPos, 1, 3);
    endPos = reshape(endPos, 1, 3);
    assert(numel(startPos) == 3 && numel(endPos) == 3, ...
        'rrtPlanner: startPos/endPos must be 1x3 world coordinates.');

    obstaclePoints = [];
    userParams = struct();
    if ~isempty(varargin)
        obstaclePoints = varargin{1};
        if ~isempty(obstaclePoints)
            assert(size(obstaclePoints, 2) == 3, ...
                'rrtPlanner: obstaclePoints must be Nx3 world coordinates.');
        end
    end
    if numel(varargin) >= 2
        userParams = varargin{2};
    end

    p = mergeStruct(defaultParams(startPos, endPos, obstaclePoints), userParams);
    if isempty(weights)
        weights = [0.1 0.1 0.1 1 1 1];
    end

    oriCandidates = buildOrientationCandidates(p.targetOrientation, p);
    stats = initRejectStats();

    treeXYZ = startPos;
    parent = 0;
    cost = 0;
    qCfg = currentConfig;

    reached = false;
    goalIdx = -1;
    iter = 0;
    timedOut = false;
    tStart = tic;

    directInfo = struct('tried', false, 'success', false, 'message', '', 'time', 0);

    if p.tryDirectFirst
        directInfo.tried = true;
        tDirect = tic;
        [directOk, directPath, directCfg, directMsg, stats] = tryDirectConnect( ...
            startPos, endPos, currentConfig, robot, ik, weights, p, obstaclePoints, oriCandidates, stats);
        directInfo.time = toc(tDirect);
        directInfo.success = directOk;
        directInfo.message = directMsg;

        if directOk
            pathPoints = directPath;
            n = size(pathPoints, 1);
            treeXYZ = pathPoints;
            parent = [0; (1:n-1)'];
            cost = [0; cumsum(vecnorm(diff(pathPoints, 1, 1), 2, 2))];
            qCfg = directCfg;
            reached = true;
            goalIdx = n;
            iter = 0;
            fprintf('--> [DP-RRT] direct-connect success, points=%d, time=%.3fs\n', ...
                n, toc(tStart));
            planInfo = buildPlanInfo(reached, treeXYZ, parent, cost, qCfg, ...
                goalIdx, iter, toc(tStart), stats, timedOut, directInfo);
            planInfo.rawPathIndices = (1:n)';
            planInfo.rawPathPoints = pathPoints;
            planInfo.rawPathConfigs = directCfg;
            planInfo.pathConfigs = directCfg;
            planInfo.pathLength = pathLength(pathPoints);
            return;
        else
            fprintf('--> [DP-RRT] direct-connect failed: %s\n', directMsg);
        end
    end

    failCount = 0;
    stallIters = 0;
    totalIter = p.maxIter;
    for iter = 1:totalIter
        if toc(tStart) > p.maxPlanTimeSec
            timedOut = true;
            break;
        end

        if mod(iter, p.progressEvery) == 1
            fprintf(['--> [DP-RRT] iter=%d/%d, nodes=%d, rej(ik=%d,pos=%d,node=%d,motion=%d,env=%d)\n'], ...
                iter, totalIter, size(treeXYZ, 1), ...
                stats.ik_failed, stats.ik_pos_error, ...
                stats.node_collision, stats.motion_collision, stats.env_edge);
            drawnow limitrate;
        end

        pg = p.PgInit * exp(-p.failDecay * failCount);
        rho = p.rhoInit * exp(-p.failDecay * failCount);

        if rand < pg
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
            vNew = unitVec((1 - rho) * vRand + rho * vGoal);
        else
            vNew = vGoal;
        end

        dMin = minDistPointToCloud(qNear, obstaclePoints);
        stepLen = adaptiveStep(dMin, p);
        qNew = qNear + stepLen * vNew;

        if ~inBounds(qNew, p.limits)
            failCount = failCount + 1;
            stats.out_of_bounds = stats.out_of_bounds + 1;
            continue;
        end

        [isValid, cfgNew, reason] = isNodeAndEdgeValid( ...
            qNear, qNew, nearIdx, qCfg, robot, ik, weights, p, obstaclePoints, oriCandidates);
        stats = bumpRejectStat(stats, reason);

        if ~isValid
            failCount = failCount + 1;
            stallIters = stallIters + 1;
            continue;
        end

        failCount = max(0, failCount - p.failRecover);
        stallIters = 0;

        treeXYZ = [treeXYZ; qNew]; %#ok<AGROW>
        parent = [parent; nearIdx]; %#ok<AGROW>
        cost = [cost; cost(nearIdx) + norm(qNew - qNear)]; %#ok<AGROW>
        qCfg = [qCfg; cfgNew]; %#ok<AGROW>
        newIdx = size(treeXYZ, 1);

        if p.allowGoalConnect
            [goalOK, goalCfg, goalReason] = isNodeAndEdgeValid( ...
                qNew, endPos, newIdx, qCfg, robot, ik, weights, p, obstaclePoints, oriCandidates);
            if goalOK
                treeXYZ = [treeXYZ; endPos]; %#ok<AGROW>
                parent = [parent; newIdx]; %#ok<AGROW>
                cost = [cost; cost(newIdx) + norm(endPos - qNew)]; %#ok<AGROW>
                qCfg = [qCfg; goalCfg]; %#ok<AGROW>
                reached = true;
                goalIdx = size(treeXYZ, 1);
                break;
            else
                stats = bumpRejectStat(stats, ['goal_' goalReason]);
            end
        end

        if norm(qNew - endPos) <= p.goalRadius
            reached = true;
            goalIdx = newIdx;
            break;
        end

        if stallIters >= p.maxStallIters
            fprintf('--> [DP-RRT] stall reached (%d iters without growth), early stop.\n', stallIters);
            break;
        end
    end

    rawPathIdx = [];
    rawPathConfigs = [];
    if reached
        rawPathIdx = backtrackIndices(parent, goalIdx);
        pathPoints = treeXYZ(rawPathIdx, :);
        rawPathConfigs = qCfg(rawPathIdx, :);
        if p.enableShortcut && size(pathPoints, 1) > 2
            pathPoints = shortcutPath(pathPoints, obstaclePoints, p);
        end
        fprintf('--> [DP-RRT] success, nodes=%d, path=%d, time=%.3fs\n', ...
            size(treeXYZ, 1), size(pathPoints, 1), toc(tStart));
    else
        pathPoints = [];
        if timedOut
            fprintf('--> [DP-RRT] timeout, iter=%d, nodes=%d, time=%.3fs\n', ...
                iter, size(treeXYZ, 1), toc(tStart));
        else
            fprintf('--> [DP-RRT] failed, iter=%d, nodes=%d, time=%.3fs\n', ...
                iter, size(treeXYZ, 1), toc(tStart));
        end
    end

    planInfo = buildPlanInfo(reached, treeXYZ, parent, cost, qCfg, ...
        goalIdx, iter, toc(tStart), stats, timedOut, directInfo);
    planInfo.rawPathIndices = rawPathIdx;
    planInfo.rawPathPoints = [];
    planInfo.rawPathConfigs = [];
    planInfo.pathConfigs = [];
    if reached
        planInfo.rawPathPoints = treeXYZ(rawPathIdx, :);
        planInfo.rawPathConfigs = rawPathConfigs;
        if ~p.enableShortcut
            planInfo.pathConfigs = rawPathConfigs;
        end
        planInfo.pathLength = pathLength(pathPoints);
    else
        planInfo.pathLength = inf;
    end
end

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
    p.maxIter = 1200;
    p.maxPlanTimeSec = 25.0;
    p.maxStallIters = 500;
    p.progressEvery = 150;

    p.goalRadius = 0.05;
    p.limits = [minXYZ(1), maxXYZ(1), minXYZ(2), maxXYZ(2), minXYZ(3), maxXYZ(3)];
    p.lambdaMax = 0.05;
    p.lambdaMin = 0.010;
    p.dSafe = 0.10;
    p.kappa = 8.0;
    p.rhoInit = 0.35;
    p.PgInit = 0.30;
    p.failDecay = 0.50;
    p.failRecover = 2;

    p.envSafeDist = 0.020;
    p.edgeSampleStep = 0.015;
    p.enableRobotCheck = true;
    p.enableEnvCheck = true;
    p.collisionFn = [];

    p.targetOrientation = eul2tform([0 0 0], 'ZYX');
    p.orientationYawOffsetsDeg = [0 180];
    p.orientationPitchOffsetsDeg = [0 -12 12];
    p.orientationRollOffsetsDeg = [0];
    p.ikCandidateTopK = 3;
    p.maxIKPosError = 0.08;
    p.maxIKPosErrorLoose = 0.15;
    p.acceptLooseIK = false;
    p.forcePositionOnlyIK = true;
    p.enableHomeSeed = true;

    p.robotCheckStep = deg2rad(2.0);
    p.allowGoalConnect = true;
    p.tryDirectFirst = true;
    p.directSamples = 24;
    p.directMaxTimeSec = 8.0;

    p.enableShortcut = true;
    p.shortcutTrials = 80;
end

function out = mergeStruct(base, ext)
    out = base;
    if isempty(ext)
        return;
    end
    f = fieldnames(ext);
    for i = 1:numel(f)
        out.(f{i}) = ext.(f{i});
    end
end

function info = buildPlanInfo(reached, treeXYZ, parent, cost, qCfg, goalIdx, iter, elapsed, stats, timedOut, directInfo)
    info = struct();
    info.isReached = reached;
    info.tree = treeXYZ;
    info.parent = parent;
    info.cost = cost;
    info.qCfg = qCfg;
    info.goalIdx = goalIdx;
    info.iter = iter;
    info.time = elapsed;
    info.rejectStats = stats;
    info.timedOut = timedOut;
    info.directInfo = directInfo;
end

function stats = initRejectStats()
    stats = struct();
    stats.ok = 0;
    stats.out_of_bounds = 0;
    stats.env_edge = 0;
    stats.ik_failed = 0;
    stats.ik_pos_error = 0;
    stats.node_collision = 0;
    stats.motion_collision = 0;
    stats.goal_env_edge = 0;
    stats.goal_ik_failed = 0;
    stats.goal_ik_pos_error = 0;
    stats.goal_node_collision = 0;
    stats.goal_motion_collision = 0;
end

function stats = bumpRejectStat(stats, key)
    if isempty(key)
        return;
    end
    if isfield(stats, key)
        stats.(key) = stats.(key) + 1;
    end
end

function [directOK, pathPoints, jointPath, message, stats] = tryDirectConnect( ...
    startPos, endPos, startCfg, robot, ik, weights, p, obstaclePoints, oriCandidates, stats)

    directOK = false;
    pathPoints = [];
    jointPath = [];
    message = 'unknown';

    t0 = tic;
    n = max(3, p.directSamples);
    t = linspace(0, 1, n)';
    linePath = startPos + t .* (endPos - startPos);

    jointPath = reshape(startCfg, 1, []);
    pathPoints = linePath(1, :);
    prevCfg = startCfg;

    for i = 2:n
        if toc(t0) > p.directMaxTimeSec
            message = sprintf('direct timeout %.2fs', toc(t0));
            return;
        end

        qNear = linePath(i-1, :);
        qNew = linePath(i, :);

        if p.enableEnvCheck && ~isempty(obstaclePoints)
            if edgeNearObstacle(qNear, qNew, obstaclePoints, p.envSafeDist, p.edgeSampleStep)
                stats.env_edge = stats.env_edge + 1;
                message = sprintf('direct env collision at step %d/%d', i, n);
                return;
            end
        end

        [ok, cfgCandidate, reason] = solveIKCandidate( ...
            qNew, prevCfg, robot, ik, weights, oriCandidates, p);
        stats = bumpRejectStat(stats, reason);
        if ~ok
            message = sprintf('direct IK fail at step %d/%d (%s)', i, n, reason);
            return;
        end

        [hitNode, ~] = invokeCollision(p, robot, cfgCandidate);
        if hitNode
            stats.node_collision = stats.node_collision + 1;
            message = sprintf('direct node collision at step %d/%d', i, n);
            return;
        end

        [hitMotion, ~] = invokeCollision(p, robot, prevCfg, cfgCandidate, p.robotCheckStep);
        if hitMotion
            stats.motion_collision = stats.motion_collision + 1;
            message = sprintf('direct motion collision at step %d/%d', i, n);
            return;
        end

        jointPath = [jointPath; cfgCandidate]; %#ok<AGROW>
        pathPoints = [pathPoints; qNew]; %#ok<AGROW>
        prevCfg = cfgCandidate;
        stats.ok = stats.ok + 1;
    end

    finalXYZ = tform2trvec(getTransform(robot, jointPath(end, :), 'shovel_tip'));
    finalErr = norm(finalXYZ - endPos);
    if finalErr > p.maxIKPosError
        message = sprintf('direct final error too large %.4f m', finalErr);
        directOK = false;
        return;
    end

    directOK = true;
    message = 'ok';
end

function [isValid, cfgOut, reason] = isNodeAndEdgeValid( ...
    qNear, qNew, nearIdx, qCfg, robot, ik, weights, p, obstaclePoints, oriCandidates)

    isValid = false;
    cfgOut = qCfg(nearIdx, :);
    reason = 'ok';

    if p.enableEnvCheck && ~isempty(obstaclePoints)
        if edgeNearObstacle(qNear, qNew, obstaclePoints, p.envSafeDist, p.edgeSampleStep)
            reason = 'env_edge';
            return;
        end
    end

    if ~p.enableRobotCheck
        isValid = true;
        cfgOut = qCfg(nearIdx, :);
        reason = 'ok';
        return;
    end

    [ikOK, cfgCandidate, ikReason] = solveIKCandidate( ...
        qNew, qCfg(nearIdx, :), robot, ik, weights, oriCandidates, p);
    if ~ikOK
        reason = ikReason;
        return;
    end

    [hitNode, ~] = invokeCollision(p, robot, cfgCandidate);
    if hitNode
        reason = 'node_collision';
        return;
    end

    [hitMotion, ~] = invokeCollision(p, robot, qCfg(nearIdx, :), cfgCandidate, p.robotCheckStep);
    if hitMotion
        reason = 'motion_collision';
        return;
    end

    isValid = true;
    cfgOut = cfgCandidate;
    reason = 'ok';
end

function [ok, cfgBest, reason] = solveIKCandidate( ...
    qNew, seedCfg, robot, ik, weights, oriCandidates, p)

    ok = false;
    cfgBest = seedCfg;
    reason = 'ik_failed';

    cfgList = [];
    posErrList = [];
    scoreList = [];
    anyIK = false;

    weightsLocal = weights;
    if p.forcePositionOnlyIK && numel(weightsLocal) >= 6
        weightsLocal(4:6) = 0;
    end

    seedList = seedCfg;
    if p.enableHomeSeed
        seedList = [seedList; homeConfiguration(robot)];
    end

    for s = 1:size(seedList, 1)
        seed = seedList(s, :);
        for i = 1:numel(oriCandidates)
            targetTform = trvec2tform(qNew) * oriCandidates{i};
            [cfgTry, ikOK] = solveIK(ik, 'shovel_tip', targetTform, weightsLocal, seed);
            if ~ikOK
                continue;
            end
            anyIK = true;

            cfgTry = unwrapConfigNearReference(cfgTry, seedCfg);
            actualXYZ = tform2trvec(getTransform(robot, cfgTry, 'shovel_tip'));
            posErr = norm(actualXYZ - qNew);

            if posErr > p.maxIKPosErrorLoose
                continue;
            end

            stepPenalty = max(abs(cfgTry - seedCfg));
            score = posErr + 0.02 * stepPenalty;
            cfgList = [cfgList; cfgTry]; %#ok<AGROW>
            posErrList = [posErrList; posErr]; %#ok<AGROW>
            scoreList = [scoreList; score]; %#ok<AGROW>
        end
    end

    if isempty(cfgList)
        if anyIK, reason = 'ik_pos_error'; else, reason = 'ik_failed'; end
        return;
    end

    [~, order] = sort(scoreList, 'ascend');
    topK = min(p.ikCandidateTopK, numel(order));
    order = order(1:topK);

    for k = 1:topK
        idx = order(k);
        if posErrList(idx) <= p.maxIKPosError
            cfgBest = cfgList(idx, :);
            ok = true;
            reason = 'ok';
            return;
        end
    end

    if p.acceptLooseIK
        idx = order(1);
        cfgBest = cfgList(idx, :);
        ok = true;
        reason = 'ok';
        return;
    end

    reason = 'ik_pos_error';
end

function oriCandidates = buildOrientationCandidates(baseOrientation, p)
    baseEul = tform2eul(baseOrientation, 'ZYX');
    yawOffsets = deg2rad(p.orientationYawOffsetsDeg(:));
    pitchOffsets = deg2rad(p.orientationPitchOffsetsDeg(:));
    rollOffsets = deg2rad(p.orientationRollOffsetsDeg(:));

    oriCandidates = cell(0, 1);
    for iy = 1:numel(yawOffsets)
        for ip = 1:numel(pitchOffsets)
            for ir = 1:numel(rollOffsets)
                eul = [ ...
                    wrapToPiLocal(baseEul(1) + yawOffsets(iy)), ...
                    wrapToPiLocal(baseEul(2) + pitchOffsets(ip)), ...
                    wrapToPiLocal(baseEul(3) + rollOffsets(ir))];
                oriCandidates{end+1, 1} = eul2tform(eul, 'ZYX'); %#ok<AGROW>
            end
        end
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
    tf = q(1) >= lim(1) && q(1) <= lim(2) && ...
         q(2) >= lim(3) && q(2) <= lim(4) && ...
         q(3) >= lim(5) && q(3) <= lim(6);
end

function dMin = minDistPointToCloud(pt, obstaclePoints)
    if isempty(obstaclePoints)
        dMin = inf;
        return;
    end
    dMin = min(vecnorm(obstaclePoints - pt, 2, 2));
end

function stepLen = adaptiveStep(dMin, p)
    if ~isfinite(dMin) || dMin >= p.dSafe
        stepLen = p.lambdaMax;
        return;
    end

    num = exp(p.kappa * dMin) - 1;
    den = exp(p.kappa * p.dSafe) - 1;
    if den <= 1e-12
        alpha = 0;
    else
        alpha = max(0, min(1, num / den));
    end
    stepLen = p.lambdaMin + (p.lambdaMax - p.lambdaMin) * alpha;
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
    for i = 1:size(samples, 1)
        if minDistPointToCloud(samples(i, :), obstaclePoints) < safeDist
            tf = true;
            return;
        end
    end
end

function idxs = backtrackIndices(parent, goalIdx)
    idx = goalIdx;
    idxs = idx;
    while idx > 1
        idx = parent(idx);
        idxs = [idx; idxs]; %#ok<AGROW>
    end
end

function path = shortcutPath(path, obstaclePoints, p)
    if size(path, 1) <= 2 || isempty(obstaclePoints)
        return;
    end

    for k = 1:p.shortcutTrials
        if size(path, 1) <= 2
            break;
        end
        i = randi([1, size(path, 1)-1]);
        j = randi([i+1, size(path, 1)]);
        if j <= i + 1
            continue;
        end
        if ~edgeNearObstacle(path(i, :), path(j, :), obstaclePoints, p.envSafeDist, p.edgeSampleStep)
            path = [path(1:i, :); path(j:end, :)];
        end
    end
end

function L = pathLength(path)
    if size(path, 1) < 2
        L = 0;
    else
        L = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end

function cfgOut = unwrapConfigNearReference(cfgIn, refCfg)
    cfgOut = cfgIn;
    for i = 1:numel(cfgIn)
        delta = cfgIn(i) - refCfg(i);
        delta = mod(delta + pi, 2*pi) - pi;
        cfgOut(i) = refCfg(i) + delta;
    end
end

function [isColliding, info] = invokeCollision(p, robot, varargin)
    if isfield(p, 'collisionFn') && ~isempty(p.collisionFn)
        [isColliding, info] = p.collisionFn(robot, varargin{:});
        return;
    end
    [isColliding, info] = robotCollision(robot, varargin{:});
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
