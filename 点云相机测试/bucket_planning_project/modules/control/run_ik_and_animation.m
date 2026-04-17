function [qExecuted, runInfo] = run_ik_and_animation( ...
    robot, endEffector, baseFrame, T_target_seq, qStart, jointLimits, collisionFcn, opts)
% run_ik_and_animation:
% Solve IK tracking with degradation policy and animate execution.
%
% Degrade policy:
%   keep orientation, reduce position displacement when IK/collision fails.

if nargin < 8
    opts = struct();
end
if ~isfield(opts, 'ikWeights'), opts.ikWeights = [1 1 1 0.7 0.7 0.7]; end
if ~isfield(opts, 'posTol'), opts.posTol = 0.04; end
if ~isfield(opts, 'oriTolDeg'), opts.oriTolDeg = 12; end
if ~isfield(opts, 'framePause'), opts.framePause = 0.015; end
if ~isfield(opts, 'render'), opts.render = true; end
if ~isfield(opts, 'ax'), opts.ax = []; end
if ~isfield(opts, 'verbose'), opts.verbose = true; end
if ~isfield(opts, 'prefixPath'), opts.prefixPath = zeros(0, numel(qStart)); end
if ~isfield(opts, 'localRrtEnabled'), opts.localRrtEnabled = true; end
if ~isfield(opts, 'localRrtMaxIter'), opts.localRrtMaxIter = 350; end
if ~isfield(opts, 'edgeStepRad'), opts.edgeStepRad = 0.08; end
if ~isfield(opts, 'interpStepRad'), opts.interpStepRad = 0.05; end
if ~isfield(opts, 'robotColor'), opts.robotColor = [0.3, 0.3, 0.3]; end
if ~isfield(opts, 'bucketPointCloud'), opts.bucketPointCloud = []; end
if ~isfield(opts, 'desiredPath'), opts.desiredPath = []; end

ik = inverseKinematics('RigidBodyTree', robot);
nTarget = size(T_target_seq, 3);
nJ = numel(qStart);

qWay = zeros(nTarget, nJ);
posErr = zeros(nTarget, 1);
oriErr = zeros(nTarget, 1);
isFallback = false(nTarget, 1);
isHold = false(nTarget, 1);

qPrev = clamp_to_limits(qStart, jointLimits);
for i = 1:nTarget
    Ttar = T_target_seq(:, :, i);
    [qTry, infoTry] = ik(endEffector, Ttar, opts.ikWeights, qPrev);
    qTry = clamp_to_limits(qTry, jointLimits);
    qTry = nearest_equivalent_to_ref(qTry, qPrev, jointLimits);

    [ok, qBest, pErr, oErr] = validateCandidate(robot, endEffector, baseFrame, Ttar, qTry, collisionFcn, opts);

    if ~ok
        isFallback(i) = true;
        [ok, qBest, pErr, oErr] = fallbackKeepOrientation( ...
            ik, robot, endEffector, baseFrame, Ttar, qPrev, jointLimits, collisionFcn, opts);
    end

    if ~ok
        % Final keep-going policy: hold previous joint to guarantee continuity.
        qBest = qPrev;
        isHold(i) = true;
        pErr = NaN;
        oErr = NaN;
    end

    qWay(i, :) = qBest;
    posErr(i) = pErr;
    oriErr(i) = oErr;
    qPrev = qBest;

    if opts.verbose && (i <= 5 || mod(i, 10) == 0 || i == nTarget)
        fprintf('[control.run_ik] %03d/%03d status=%s fallback=%d hold=%d posErr=%.4f oriErr=%.2f\n', ...
            i, nTarget, string(infoTry.Status), isFallback(i), isHold(i), pErr, oErr);
    end
end

qExecuted = opts.prefixPath;
if isempty(qExecuted)
    qExecuted = qStart;
end

for i = 1:nTarget
    qA = qExecuted(end, :);
    qB = qWay(i, :);
    seg = connectSegment(qA, qB, jointLimits, collisionFcn, opts);
    if size(seg, 1) > 1
        qExecuted = [qExecuted; seg(2:end, :)]; %#ok<AGROW>
    end
end

tipTrail = nan(size(qExecuted, 1), 3);
if opts.render
    ax = opts.ax;
    if isempty(ax) || ~ishandle(ax)
        fig = figure('Name', 'RRT* + IK Execution', 'Color', 'w', 'Position', [100, 80, 1300, 780]); %#ok<NASGU>
        ax = axes('Parent', gcf);
    end
    cla(ax);
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    view(ax, 125, 24);
    hold(ax, 'on');

    if ~isempty(opts.bucketPointCloud)
        pcshow(opts.bucketPointCloud, 'Parent', ax, 'MarkerSize', 35);
    end
    if ~isempty(opts.desiredPath)
        plot3(ax, opts.desiredPath(:, 1), opts.desiredPath(:, 2), opts.desiredPath(:, 3), ...
            'r-', 'LineWidth', 2.2);
    end

    for i = 1:size(qExecuted, 1)
        Ttip = getTransform(robot, qExecuted(i, :), endEffector, baseFrame);
        tipTrail(i, :) = tform2trvec(Ttip);
        show(robot, qExecuted(i, :), 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);
        hold(ax, 'on');
        if ~isempty(opts.bucketPointCloud)
            pcshow(opts.bucketPointCloud, 'Parent', ax, 'MarkerSize', 35);
        end
        if ~isempty(opts.desiredPath)
            plot3(ax, opts.desiredPath(:, 1), opts.desiredPath(:, 2), opts.desiredPath(:, 3), ...
                'r-', 'LineWidth', 2.2);
        end
        plot3(ax, tipTrail(1:i, 1), tipTrail(1:i, 2), tipTrail(1:i, 3), 'b-', 'LineWidth', 1.6);
        title(ax, sprintf('Execution %d/%d', i, size(qExecuted, 1)));
        drawnow;
        pause(opts.framePause);
    end
end

runInfo = struct();
runInfo.nTarget = nTarget;
runInfo.nExecuted = size(qExecuted, 1);
runInfo.nFallback = sum(isFallback);
runInfo.nHold = sum(isHold);
runInfo.posErrMean = safeMean(posErr);
runInfo.posErrMax = safeMax(posErr);
runInfo.oriErrMean = safeMean(oriErr);
runInfo.oriErrMax = safeMax(oriErr);
runInfo.qWay = qWay;
runInfo.tipTrail = tipTrail;
end

function [ok, qUse, posErr, oriErrDeg] = validateCandidate( ...
    robot, endEffector, baseFrame, Ttar, qTry, collisionFcn, opts)
ok = false;
qUse = qTry;
Tnow = getTransform(robot, qTry, endEffector, baseFrame);
pTar = tform2trvec(Ttar);
pNow = tform2trvec(Tnow);
Rtar = tform2rotm(Ttar);
Rnow = tform2rotm(Tnow);

posErr = norm(pNow - pTar);
oriErrDeg = rotm_geodesic_deg(Rnow, Rtar);
if collisionFcn(qTry)
    return;
end
if posErr > opts.posTol || oriErrDeg > opts.oriTolDeg
    return;
end
ok = true;
end

function [ok, qBest, posBest, oriBest] = fallbackKeepOrientation( ...
    ik, robot, endEffector, baseFrame, Ttar, qPrev, jointLimits, collisionFcn, opts)

ok = false;
qBest = qPrev;
posBest = inf;
oriBest = inf;

pTar = tform2trvec(Ttar);
Rtar = tform2rotm(Ttar);
Tprev = getTransform(robot, qPrev, endEffector, baseFrame);
pPrev = tform2trvec(Tprev);

alphaList = [0.90, 0.75, 0.60, 0.45, 0.30, 0.20, 0.10];
for i = 1:numel(alphaList)
    a = alphaList(i);
    pAdj = pPrev + a * (pTar - pPrev);
    Tadj = eye(4);
    Tadj(1:3, 1:3) = Rtar;
    Tadj(1:3, 4) = pAdj(:);

    [qTry, ~] = ik(endEffector, Tadj, opts.ikWeights, qPrev);
    qTry = clamp_to_limits(qTry, jointLimits);
    qTry = nearest_equivalent_to_ref(qTry, qPrev, jointLimits);

    Tnow = getTransform(robot, qTry, endEffector, baseFrame);
    pNow = tform2trvec(Tnow);
    Rnow = tform2rotm(Tnow);
    posErr = norm(pNow - pAdj);
    oriErr = rotm_geodesic_deg(Rnow, Rtar);

    if ~collisionFcn(qTry) && (oriErr <= opts.oriTolDeg)
        qBest = qTry;
        posBest = posErr;
        oriBest = oriErr;
        ok = true;
        return;
    end
end
end

function qSeg = connectSegment(qA, qB, jointLimits, collisionFcn, opts)
qSeg = linearInterpolate(qA, qB, opts.interpStepRad);
if pathCollisionFree(qSeg, collisionFcn)
    return;
end

if opts.localRrtEnabled
    plannerOpts = struct( ...
        'maxIter', opts.localRrtMaxIter, ...
        'goalBias', 0.25, ...
        'stepSize', 0.25, ...
        'nearRadius', 0.65, ...
        'goalThresh', 0.18, ...
        'edgeStep', opts.edgeStepRad, ...
        'shortcutIters', 50, ...
        'verbose', false);
    try
        [qPlan, ~] = plan_rrtstar_joint_path(qA, qB, jointLimits, collisionFcn, plannerOpts);
        if size(qPlan, 1) >= 2 && pathCollisionFree(qPlan, collisionFcn)
            qSeg = densifyPath(qPlan, opts.interpStepRad);
            return;
        end
    catch
        % Keep-going: ignore local planner error and fallback below.
    end
end

% Keep-going fallback: attempt clamped direct interpolation.
qSeg = densifyPath([qA; qB], opts.interpStepRad);
for i = 1:size(qSeg, 1)
    if collisionFcn(qSeg(i, :))
        qSeg(i, :) = qA;
    end
end
end

function q = linearInterpolate(qA, qB, stepRad)
d = norm(qB - qA);
n = max(2, ceil(d / max(stepRad, 1e-5)));
alpha = linspace(0, 1, n)';
q = (1 - alpha) * qA + alpha * qB;
end

function qOut = densifyPath(qPath, stepRad)
if size(qPath, 1) < 2
    qOut = qPath;
    return;
end
qOut = qPath(1, :);
for i = 1:(size(qPath, 1) - 1)
    seg = linearInterpolate(qPath(i, :), qPath(i + 1, :), stepRad);
    qOut = [qOut; seg(2:end, :)]; %#ok<AGROW>
end
end

function ok = pathCollisionFree(qPath, collisionFcn)
ok = true;
for i = 1:size(qPath, 1)
    if collisionFcn(qPath(i, :))
        ok = false;
        return;
    end
end
end

function v = safeMean(x)
x = x(~isnan(x));
if isempty(x)
    v = NaN;
else
    v = mean(x);
end
end

function v = safeMax(x)
x = x(~isnan(x));
if isempty(x)
    v = NaN;
else
    v = max(x);
end
end
