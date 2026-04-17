function [reachRadius, reachInfo] = estimate_reach_radius_from_urdf(robot, baseLink, endEffector, opts)
% estimate_reach_radius_from_urdf:
% Estimate max reachable radius using FK envelope sampling.

if nargin < 4
    opts = struct();
end
if ~isfield(opts, 'sampleCount') || isempty(opts.sampleCount)
    opts.sampleCount = 3000;
end
if ~isfield(opts, 'seed') || isempty(opts.seed)
    opts.seed = 42;
end
if ~isfield(opts, 'margin') || isempty(opts.margin)
    opts.margin = 0.0;
end

jointLimits = collect_joint_limits(robot);
nJ = size(jointLimits, 1);

if nJ == 0
    reachRadius = 1.0;
    reachInfo = struct('sampleCount', 0, 'maxDist', reachRadius, 'p95', reachRadius, ...
        'p99', reachRadius, 'qAtMax', []);
    return;
end

rng(opts.seed);
qSamples = zeros(opts.sampleCount + 2, nJ);
qHome = clamp_to_limits(homeConfiguration(robot), jointLimits);
qSamples(1, :) = qHome;
qSamples(2, :) = zeros(1, nJ);
for i = 3:size(qSamples, 1)
    q = jointLimits(:, 1)' + rand(1, nJ) .* (jointLimits(:, 2)' - jointLimits(:, 1)');
    qSamples(i, :) = clamp_to_limits(q, jointLimits);
end

dists = zeros(size(qSamples, 1), 1);
for i = 1:size(qSamples, 1)
    T = getTransform(robot, qSamples(i, :), endEffector, baseLink);
    p = tform2trvec(T);
    dists(i) = norm(p);
end

[maxDist, idxMax] = max(dists);
reachRadius = maxDist + opts.margin;

reachInfo = struct();
reachInfo.sampleCount = size(qSamples, 1);
reachInfo.maxDist = maxDist;
reachInfo.reachRadius = reachRadius;
reachInfo.p95 = quantile(dists, 0.95);
reachInfo.p99 = quantile(dists, 0.99);
reachInfo.qAtMax = qSamples(idxMax, :);

fprintf('[common.reach] samples=%d max=%.4f p95=%.4f p99=%.4f radius=%.4f\n', ...
    reachInfo.sampleCount, reachInfo.maxDist, reachInfo.p95, reachInfo.p99, reachRadius);
end

