function [qPath, info] = plan_dprrtstar_joint_path_cuda(qStart, qGoal, jointLimits, collisionFcn, opts)
% plan_dprrtstar_joint_path_cuda:
% DP-RRT* (paper-inspired) planner in joint space with optional CUDA NN accel.
%
% Core mechanisms adapted from Algo_DP_RRT_3D:
%   1) Dynamic goal bias Pg(fail_count) to escape traps.
%   2) Dynamic attraction rho(fail_count) to blend random and goal direction.
%   3) Adaptive step lambda(d_min) from obstacle/limit clearance.

if nargin < 5
    opts = struct();
end
if ~isfield(opts, 'maxIter'), opts.maxIter = 2200; end
if ~isfield(opts, 'goalThresh'), opts.goalThresh = 0.20; end
if ~isfield(opts, 'nearRadius'), opts.nearRadius = 0.65; end
if ~isfield(opts, 'edgeStep'), opts.edgeStep = 0.08; end
if ~isfield(opts, 'shortcutIters'), opts.shortcutIters = 100; end
if ~isfield(opts, 'verbose'), opts.verbose = true; end
if ~isfield(opts, 'logEvery'), opts.logEvery = 100; end
if ~isfield(opts, 'useGPU'), opts.useGPU = true; end
if ~isfield(opts, 'forceGPU'), opts.forceGPU = false; end

% DP-RRT* parameters (joint-space adaptation).
if ~isfield(opts, 'goalBiasInit'), opts.goalBiasInit = 0.28; end
if ~isfield(opts, 'goalBiasMin'), opts.goalBiasMin = 0.03; end
if ~isfield(opts, 'rhoInit'), opts.rhoInit = 0.50; end
if ~isfield(opts, 'decayRate'), opts.decayRate = 0.50; end
if ~isfield(opts, 'lambdaMax'), opts.lambdaMax = 0.24; end
if ~isfield(opts, 'lambdaMin'), opts.lambdaMin = 0.08; end
if ~isfield(opts, 'dSafe'), opts.dSafe = 0.35; end
if ~isfield(opts, 'stepKappa'), opts.stepKappa = 3.00; end
if ~isfield(opts, 'failRecovery'), opts.failRecovery = 2; end
if ~isfield(opts, 'maxCollisionSamples'), opts.maxCollisionSamples = 1500; end

qStart = clamp_to_limits(qStart, jointLimits);
qGoal = clamp_to_limits(qGoal, jointLimits);

if collisionFcn(qStart)
    error('[control.dp_rrtstar_cuda] qStart is colliding.');
end
if collisionFcn(qGoal)
    error('[control.dp_rrtstar_cuda] qGoal is colliding.');
end

gpuState = initGpuState(opts);
nJ = numel(qStart);
nodes = struct('q', qStart, 'parent', 0, 'cost', 0);
goalNodeIdx = 0;

failCount = 0;
collisionSamples = zeros(0, nJ);
iterUsed = opts.maxIter;
tPlan = tic;

if opts.verbose
    fprintf(['[control.dp_rrtstar_cuda] start nJ=%d maxIter=%d nearR=%.2f thresh=%.2f ' ...
        'goalBiasInit=%.2f rhoInit=%.2f lambda=[%.3f, %.3f] dSafe=%.3f gpu=%d\n'], ...
        nJ, opts.maxIter, opts.nearRadius, opts.goalThresh, ...
        opts.goalBiasInit, opts.rhoInit, opts.lambdaMin, opts.lambdaMax, opts.dSafe, gpuState.enabled);
    if gpuState.enabled
        fprintf('[control.dp_rrtstar_cuda] CUDA device: %s\n', gpuState.name);
    else
        fprintf('[control.dp_rrtstar_cuda] GPU fallback CPU: %s\n', gpuState.reason);
    end
end

for it = 1:opts.maxIter
    iterUsed = it;
    pg = max(opts.goalBiasMin, opts.goalBiasInit * exp(-opts.decayRate * failCount));
    rho = max(0.0, min(1.0, opts.rhoInit * exp(-opts.decayRate * failCount)));

    if rand < pg
        qRand = qGoal;
    else
        qRand = sampleInLimits(jointLimits);
    end

    [idxNear, qNear] = nearestNodeAccel(nodes, qRand, gpuState);

    vRand = normalizeVec(qRand - qNear);
    vGoal = normalizeVec(qGoal - qNear);
    if norm(qRand - qGoal) > 1e-6
        vNew = normalizeVec((1 - rho) * vRand + rho * vGoal);
    else
        vNew = vGoal;
    end
    if norm(vNew) < 1e-9
        vNew = vGoal;
    end
    if norm(vNew) < 1e-9
        vNew = normalizeVec(randn(1, nJ));
    end

    dMin = estimateClearanceJoint(qNear, collisionSamples, jointLimits);
    lambda = adaptiveStep(dMin, opts.dSafe, opts.lambdaMin, opts.lambdaMax, opts.stepKappa);
    qNew = clamp_to_limits(qNear + lambda * vNew, jointLimits);

    if norm(qNew - qNear) < 1e-9
        failCount = failCount + 1;
        continue;
    end

    [okEdge, qHit] = edgeCollisionFreeWithHit(qNear, qNew, opts.edgeStep, collisionFcn);
    if ~okEdge
        failCount = failCount + 1;
        collisionSamples = appendCollisionSample(collisionSamples, qHit, opts.maxCollisionSamples);
        continue;
    end

    failCount = max(0, failCount - opts.failRecovery);

    nearIdx = radiusNeighborsAccel(nodes, qNew, opts.nearRadius, gpuState);
    parentIdx = idxNear;
    minCost = nodes(idxNear).cost + norm(qNew - qNear);

    for k = 1:numel(nearIdx)
        idx = nearIdx(k);
        qCand = nodes(idx).q;
        c = nodes(idx).cost + norm(qNew - qCand);
        if c < minCost
            [okParent, ~] = edgeCollisionFreeWithHit(qCand, qNew, opts.edgeStep, collisionFcn);
            if okParent
                parentIdx = idx;
                minCost = c;
            end
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
        if cThrough < nodes(idx).cost
            [okRewire, qHit] = edgeCollisionFreeWithHit(qNew, qCand, opts.edgeStep, collisionFcn);
            if okRewire
                nodes(idx).parent = idxNew;
                nodes(idx).cost = cThrough;
            else
                collisionSamples = appendCollisionSample(collisionSamples, qHit, opts.maxCollisionSamples);
            end
        end
    end

    [okGoal, qHitGoal] = edgeCollisionFreeWithHit(qNew, qGoal, opts.edgeStep, collisionFcn);
    if norm(qNew - qGoal) <= opts.goalThresh && okGoal
        goalNode.q = qGoal; %#ok<STRNU>
        goalNode.parent = idxNew;
        goalNode.cost = nodes(idxNew).cost + norm(qGoal - qNew);
        nodes(end + 1) = goalNode; %#ok<AGROW>
        goalNodeIdx = numel(nodes);
        if opts.verbose
            fprintf('[control.dp_rrtstar_cuda] Goal reached at iter=%d nodes=%d cost=%.3f fail=%d\n', ...
                it, numel(nodes), nodes(goalNodeIdx).cost, failCount);
        end
        break;
    elseif ~okGoal
        collisionSamples = appendCollisionSample(collisionSamples, qHitGoal, opts.maxCollisionSamples);
    end

    if opts.verbose && mod(it, max(1, opts.logEvery)) == 0
        [~, qNearGoal] = nearestNodeAccel(nodes, qGoal, gpuState);
        fprintf(['[control.dp_rrtstar_cuda] iter=%d nodes=%d nearGoalDist=%.3f fail=%d ' ...
            'Pg=%.3f rho=%.3f lambda=%.3f dmin=%.3f collSamples=%d\n'], ...
            it, numel(nodes), norm(qNearGoal - qGoal), failCount, pg, rho, lambda, dMin, size(collisionSamples, 1));
    end
end

info = struct();
info.success = (goalNodeIdx ~= 0);
info.numNodes = numel(nodes);
info.maxIter = opts.maxIter;
info.iterUsed = iterUsed;
info.gpu = gpuState;
info.algorithm = 'dp_rrtstar_cuda';
info.failCountFinal = failCount;
info.collisionSamples = size(collisionSamples, 1);

if goalNodeIdx == 0
    [idxBest, qBest] = nearestNodeAccel(nodes, qGoal, gpuState);
    [okFallback, qHit] = edgeCollisionFreeWithHit(qBest, qGoal, opts.edgeStep, collisionFcn);
    if ~okFallback
        qPath = qStart;
        info.message = 'DP-RRT* failed to connect goal and fallback edge collides.';
        info.bestNode = idxBest;
        info.elapsedSec = toc(tPlan);
        collisionSamples = appendCollisionSample(collisionSamples, qHit, opts.maxCollisionSamples); %#ok<NASGU>
        if opts.verbose
            fprintf('[control.dp_rrtstar_cuda] failed: nearest fallback edge collides (idxBest=%d)\n', idxBest);
        end
        return;
    end
    nodes(end + 1) = struct('q', qGoal, 'parent', idxBest, ...
        'cost', nodes(idxBest).cost + norm(qGoal - qBest)); %#ok<AGROW>
    goalNodeIdx = numel(nodes);
    info.message = 'DP-RRT* reached fallback via nearest node.';
else
    info.message = 'DP-RRT* success.';
end

qPath = backtracePath(nodes, goalNodeIdx);
qPath = shortcutPath(qPath, opts.shortcutIters, opts.edgeStep, collisionFcn);
info.pathLength = pathLength(qPath);
info.numPathPoints = size(qPath, 1);
info.elapsedSec = toc(tPlan);

if opts.verbose
    fprintf(['[control.dp_rrtstar_cuda] done success=%d nodes=%d iter=%d pathPts=%d ' ...
        'pathLen=%.3f t=%.3fs gpu=%d failFinal=%d\n'], ...
        info.success, info.numNodes, info.iterUsed, info.numPathPoints, ...
        info.pathLength, info.elapsedSec, gpuState.enabled, info.failCountFinal);
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
        error('[control.dp_rrtstar_cuda] forceGPU=true but GPU toolbox unavailable.');
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
        error('[control.dp_rrtstar_cuda] forceGPU=true but no CUDA GPU detected.');
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
        error('[control.dp_rrtstar_cuda] forceGPU=true but gpuDevice failed: %s', ME.message);
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

function v = normalizeVec(v)
n = norm(v);
if n < 1e-12
    return;
end
v = v / n;
end

function dMin = estimateClearanceJoint(q, collisionSamples, jointLimits)
dLim = min([q - jointLimits(:, 1)'; jointLimits(:, 2)' - q], [], 'all');
if isempty(dLim) || ~isfinite(dLim)
    dLim = inf;
end
if isempty(collisionSamples)
    dObs = inf;
else
    dObs = min(vecnorm(collisionSamples - q, 2, 2));
end
dMin = max(0, min(dLim, dObs));
end

function lambda = adaptiveStep(dMin, dSafe, lambdaMin, lambdaMax, kappa)
if ~isfinite(dMin) || dMin >= dSafe
    lambda = lambdaMax;
    return;
end
if dSafe <= 1e-9
    lambda = lambdaMin;
    return;
end
num = exp(kappa * max(0, dMin)) - 1;
den = exp(kappa * dSafe) - 1;
if abs(den) < 1e-12
    ratio = max(0, min(1, dMin / dSafe));
else
    ratio = max(0, min(1, num / den));
end
lambda = lambdaMin + (lambdaMax - lambdaMin) * ratio;
lambda = max(lambdaMin, min(lambdaMax, lambda));
end

function [ok, qHit] = edgeCollisionFreeWithHit(qA, qB, step, collisionFcn)
dist = norm(qB - qA);
n = max(2, ceil(dist / max(step, 1e-5)));
ok = true;
qHit = [];
for i = 0:n
    a = i / n;
    q = (1 - a) * qA + a * qB;
    if collisionFcn(q)
        ok = false;
        qHit = q;
        return;
    end
end
end

function samples = appendCollisionSample(samples, qHit, maxN)
if isempty(qHit)
    return;
end
samples = [samples; qHit]; %#ok<AGROW>
if size(samples, 1) > maxN
    samples = samples(end - maxN + 1:end, :);
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
    [okEdge, ~] = edgeCollisionFreeWithHit(qOut(i, :), qOut(j, :), edgeStep, collisionFcn);
    if okEdge
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
