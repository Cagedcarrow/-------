function [qPath, info] = plan_rrtstar_joint_path(qStart, qGoal, jointLimits, collisionFcn, opts)
% plan_rrtstar_joint_path:
% Joint-space RRT* planner with collision checks.

if nargin < 5
    opts = struct();
end
if ~isfield(opts, 'maxIter'), opts.maxIter = 1800; end
if ~isfield(opts, 'goalBias'), opts.goalBias = 0.18; end
if ~isfield(opts, 'stepSize'), opts.stepSize = 0.22; end
if ~isfield(opts, 'nearRadius'), opts.nearRadius = 0.55; end
if ~isfield(opts, 'goalThresh'), opts.goalThresh = 0.20; end
if ~isfield(opts, 'edgeStep'), opts.edgeStep = 0.08; end
if ~isfield(opts, 'shortcutIters'), opts.shortcutIters = 100; end
if ~isfield(opts, 'verbose'), opts.verbose = true; end

qStart = clamp_to_limits(qStart, jointLimits);
qGoal = clamp_to_limits(qGoal, jointLimits);

if collisionFcn(qStart)
    error('[control.rrtstar] qStart is colliding.');
end
if collisionFcn(qGoal)
    error('[control.rrtstar] qGoal is colliding.');
end

nJ = numel(qStart);
nodes = struct('q', qStart, 'parent', 0, 'cost', 0);
goalNode = 0;

for it = 1:opts.maxIter
    if rand < opts.goalBias
        qRand = qGoal;
    else
        qRand = sampleInLimits(jointLimits);
    end

    [idxNear, qNear] = nearestNode(nodes, qRand);
    qNew = steer(qNear, qRand, opts.stepSize, jointLimits);

    if ~edgeCollisionFree(qNear, qNew, opts.edgeStep, collisionFcn)
        continue;
    end

    nearIdx = radiusNeighbors(nodes, qNew, opts.nearRadius);
    parentIdx = idxNear;
    minCost = nodes(idxNear).cost + norm(qNew - qNear);

    for k = 1:numel(nearIdx)
        idx = nearIdx(k);
        qCand = nodes(idx).q;
        c = nodes(idx).cost + norm(qNew - qCand);
        if c < minCost && edgeCollisionFree(qCand, qNew, opts.edgeStep, collisionFcn)
            parentIdx = idx;
            minCost = c;
        end
    end

    newNode.q = qNew;
    newNode.parent = parentIdx;
    newNode.cost = minCost;
    nodes(end + 1) = newNode; %#ok<AGROW>
    idxNew = numel(nodes);

    % Rewire.
    for k = 1:numel(nearIdx)
        idx = nearIdx(k);
        if idx == parentIdx
            continue;
        end
        qCand = nodes(idx).q;
        cThrough = nodes(idxNew).cost + norm(qCand - qNew);
        if cThrough < nodes(idx).cost && edgeCollisionFree(qNew, qCand, opts.edgeStep, collisionFcn)
            nodes(idx).parent = idxNew;
            nodes(idx).cost = cThrough;
        end
    end

    if norm(qNew - qGoal) <= opts.goalThresh && edgeCollisionFree(qNew, qGoal, opts.edgeStep, collisionFcn)
        goalNode.q = qGoal;
        goalNode.parent = idxNew;
        goalNode.cost = nodes(idxNew).cost + norm(qGoal - qNew);
        nodes(end + 1) = goalNode; %#ok<AGROW>
        goalNode = numel(nodes);
        if opts.verbose
            fprintf('[control.rrtstar] Goal reached at iter=%d nodes=%d cost=%.3f\n', ...
                it, numel(nodes), nodes(goalNode).cost);
        end
        break;
    end

    if opts.verbose && mod(it, 200) == 0
        [~, dNearGoal] = nearestNode(nodes, qGoal);
        fprintf('[control.rrtstar] iter=%d nodes=%d nearGoalDist=%.3f\n', ...
            it, numel(nodes), norm(dNearGoal - qGoal));
    end
end

info = struct();
info.success = (goalNode ~= 0);
info.numNodes = numel(nodes);
info.maxIter = opts.maxIter;

if goalNode == 0
    [idxBest, qBest] = nearestNode(nodes, qGoal);
    if ~edgeCollisionFree(qBest, qGoal, opts.edgeStep, collisionFcn)
        qPath = qStart;
        info.message = 'RRT* failed to connect goal and fallback edge collides.';
        info.bestNode = idxBest;
        return;
    end
    nodes(end + 1) = struct('q', qGoal, 'parent', idxBest, ...
        'cost', nodes(idxBest).cost + norm(qGoal - qBest)); %#ok<AGROW>
    goalNode = numel(nodes);
    info.message = 'RRT* reached fallback via nearest node.';
else
    info.message = 'RRT* success.';
end

qPath = backtracePath(nodes, goalNode);
qPath = shortcutPath(qPath, opts.shortcutIters, opts.edgeStep, collisionFcn);
info.pathLength = pathLength(qPath);
info.numPathPoints = size(qPath, 1);
end

function q = sampleInLimits(lim)
q = lim(:, 1)' + rand(1, size(lim, 1)) .* (lim(:, 2)' - lim(:, 1)');
end

function [idx, qNear] = nearestNode(nodes, q)
allQ = vertcat(nodes.q);
[~, idx] = min(sum((allQ - q).^2, 2));
qNear = nodes(idx).q;
end

function idx = radiusNeighbors(nodes, q, r)
allQ = vertcat(nodes.q);
d = sqrt(sum((allQ - q).^2, 2));
idx = find(d <= r);
if isempty(idx)
    [idx, ~] = nearestNode(nodes, q); %#ok<ASGLU>
end
end

function qNew = steer(qFrom, qTo, stepSize, lim)
d = qTo - qFrom;
nd = norm(d);
if nd < 1e-12
    qNew = qFrom;
else
    alpha = min(1.0, stepSize / nd);
    qNew = qFrom + alpha * d;
end
qNew = clamp_to_limits(qNew, lim);
end

function ok = edgeCollisionFree(qA, qB, step, collisionFcn)
dist = norm(qB - qA);
n = max(2, ceil(dist / max(step, 1e-5)));
ok = true;
for i = 0:n
    a = i / n;
    q = (1 - a) * qA + a * qB;
    if collisionFcn(q)
        ok = false;
        return;
    end
end
end

function qPath = backtracePath(nodes, idxGoal)
idx = idxGoal;
qPath = zeros(0, numel(nodes(1).q));
while idx ~= 0
    qPath = [nodes(idx).q; qPath]; %#ok<AGROW>
    idx = nodes(idx).parent;
end
end

function qOut = shortcutPath(qIn, nIters, edgeStep, collisionFcn)
qOut = qIn;
if size(qOut, 1) < 3
    return;
end
for k = 1:nIters
    n = size(qOut, 1);
    if n < 3
        break;
    end
    i = randi([1, n - 2]);
    j = randi([i + 2, n]);
    if edgeCollisionFree(qOut(i, :), qOut(j, :), edgeStep, collisionFcn)
        qOut = [qOut(1:i, :); qOut(j:end, :)];
    end
end
end

function L = pathLength(qPath)
if size(qPath, 1) < 2
    L = 0;
    return;
end
d = diff(qPath, 1, 1);
L = sum(vecnorm(d, 2, 2));
end

