function [qPath, info] = plan_rrtstar_joint_path_cuda(qStart, qGoal, jointLimits, collisionFcn, opts)
% plan_rrtstar_joint_path_cuda:
% RRT* planner with optional CUDA acceleration for NN/radius queries.
%
% Notes:
%   - Collision checks remain on CPU.
%   - GPU is used only for distance-heavy operations.

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
if ~isfield(opts, 'logEvery'), opts.logEvery = 100; end
if ~isfield(opts, 'useGPU'), opts.useGPU = true; end
if ~isfield(opts, 'forceGPU'), opts.forceGPU = false; end

qStart = clamp_to_limits(qStart, jointLimits);
qGoal = clamp_to_limits(qGoal, jointLimits);

if collisionFcn(qStart)
    error('[control.rrtstar_cuda] qStart is colliding.');
end
if collisionFcn(qGoal)
    error('[control.rrtstar_cuda] qGoal is colliding.');
end

gpuState = initGpuState(opts);

nJ = numel(qStart);
nodes = struct('q', qStart, 'parent', 0, 'cost', 0);
goalNodeIdx = 0;

if opts.verbose
    fprintf(['[control.rrtstar_cuda] start nJ=%d maxIter=%d step=%.3f goalBias=%.2f ' ...
        'nearR=%.2f thresh=%.2f gpu=%d\n'], ...
        nJ, opts.maxIter, opts.stepSize, opts.goalBias, opts.nearRadius, opts.goalThresh, gpuState.enabled);
    if gpuState.enabled
        fprintf('[control.rrtstar_cuda] CUDA device: %s\n', gpuState.name);
    else
        fprintf('[control.rrtstar_cuda] GPU fallback CPU: %s\n', gpuState.reason);
    end
end

for it = 1:opts.maxIter
    if rand < opts.goalBias
        qRand = qGoal;
    else
        qRand = sampleInLimits(jointLimits);
    end

    [idxNear, qNear] = nearestNodeAccel(nodes, qRand, gpuState);
    qNew = steer(qNear, qRand, opts.stepSize, jointLimits);

    if ~edgeCollisionFree(qNear, qNew, opts.edgeStep, collisionFcn)
        continue;
    end

    nearIdx = radiusNeighborsAccel(nodes, qNew, opts.nearRadius, gpuState);
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
        goalNode = struct();
        goalNode.q = qGoal;
        goalNode.parent = idxNew;
        goalNode.cost = nodes(idxNew).cost + norm(qGoal - qNew);
        nodes(end + 1) = goalNode; %#ok<AGROW>
        goalNodeIdx = numel(nodes);
        if opts.verbose
            fprintf('[control.rrtstar_cuda] Goal reached at iter=%d nodes=%d cost=%.3f\n', ...
                it, numel(nodes), nodes(goalNodeIdx).cost);
        end
        break;
    end

    if opts.verbose && mod(it, max(1, opts.logEvery)) == 0
        [~, qNearGoal] = nearestNodeAccel(nodes, qGoal, gpuState);
        fprintf('[control.rrtstar_cuda] iter=%d nodes=%d nearGoalDist=%.3f\n', ...
            it, numel(nodes), norm(qNearGoal - qGoal));
    end
end

info = struct();
info.success = (goalNodeIdx ~= 0);
info.numNodes = numel(nodes);
info.maxIter = opts.maxIter;
info.gpu = gpuState;

if goalNodeIdx == 0
    [idxBest, qBest] = nearestNodeAccel(nodes, qGoal, gpuState);
    if ~edgeCollisionFree(qBest, qGoal, opts.edgeStep, collisionFcn)
        qPath = qStart;
        info.message = 'RRT* failed to connect goal and fallback edge collides.';
        info.bestNode = idxBest;
        if opts.verbose
            fprintf('[control.rrtstar_cuda] failed: nearest fallback edge collides (idxBest=%d)\n', idxBest);
        end
        return;
    end
    nodes(end + 1) = struct('q', qGoal, 'parent', idxBest, ...
        'cost', nodes(idxBest).cost + norm(qGoal - qBest)); %#ok<AGROW>
    goalNodeIdx = numel(nodes);
    info.message = 'RRT* reached fallback via nearest node.';
else
    info.message = 'RRT* success.';
end

qPath = backtracePath(nodes, goalNodeIdx);
qPath = shortcutPath(qPath, opts.shortcutIters, opts.edgeStep, collisionFcn);
info.pathLength = pathLength(qPath);
info.numPathPoints = size(qPath, 1);
if opts.verbose
    fprintf('[control.rrtstar_cuda] done success=%d nodes=%d pathPts=%d pathLen=%.3f gpu=%d\n', ...
        info.success, info.numNodes, info.numPathPoints, info.pathLength, gpuState.enabled);
end
end

function gpuState = initGpuState(opts)
gpuState = struct('enabled', false, 'name', '', 'reason', 'gpu disabled by opts', ...
    'requested', opts.useGPU, 'force', opts.forceGPU);

if ~opts.useGPU
    return;
end

if exist('gpuDeviceCount', 'file') ~= 2 || exist('gpuDevice', 'file') ~= 2
    gpuState.reason = 'Parallel Computing Toolbox / GPU functions unavailable';
    if opts.forceGPU
        error('[control.rrtstar_cuda] forceGPU=true but GPU toolbox unavailable.');
    end
    return;
end

count = 0;
try
    count = gpuDeviceCount("available");
catch
    try
        count = gpuDeviceCount;
    catch
        count = 0;
    end
end

if count < 1
    gpuState.reason = 'No CUDA GPU detected';
    if opts.forceGPU
        error('[control.rrtstar_cuda] forceGPU=true but no CUDA GPU detected.');
    end
    return;
end

try
    g = gpuDevice;
    gpuState.enabled = true;
    gpuState.name = g.Name;
    gpuState.reason = 'ok';
catch ME
    gpuState.reason = sprintf('gpuDevice init failed: %s', ME.message);
    if opts.forceGPU
        error('[control.rrtstar_cuda] forceGPU=true but gpuDevice failed: %s', ME.message);
    end
end
end

function q = sampleInLimits(lim)
q = lim(:, 1)' + rand(1, size(lim, 1)) .* (lim(:, 2)' - lim(:, 1)');
end

function [idx, qNear] = nearestNodeAccel(nodes, q, gpuState)
allQ = vertcat(nodes.q);
if gpuState.enabled
    qGpu = gpuArray(single(q));
    allQGpu = gpuArray(single(allQ));
    d2 = sum((allQGpu - qGpu).^2, 2);
    [~, idxGpu] = min(d2);
    idx = gather(idxGpu);
else
    [~, idx] = min(sum((allQ - q).^2, 2));
end
qNear = nodes(idx).q;
end

function idx = radiusNeighborsAccel(nodes, q, r, gpuState)
allQ = vertcat(nodes.q);
r2 = r * r;
if gpuState.enabled
    qGpu = gpuArray(single(q));
    allQGpu = gpuArray(single(allQ));
    d2 = sum((allQGpu - qGpu).^2, 2);
    mask = gather(d2 <= single(r2));
    idx = find(mask);
else
    d2 = sum((allQ - q).^2, 2);
    idx = find(d2 <= r2);
end
if isempty(idx)
    [idx, ~] = nearestNodeAccel(nodes, q, gpuState); %#ok<ASGLU>
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
