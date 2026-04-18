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
if ~isfield(opts, 'maxConsecutiveLocalFail'), opts.maxConsecutiveLocalFail = 8; end
if ~isfield(opts, 'maxTotalLocalFail'), opts.maxTotalLocalFail = 40; end
if ~isfield(opts, 'maxRunSeconds'), opts.maxRunSeconds = 90; end
if ~isfield(opts, 'maxIkSeconds'), opts.maxIkSeconds = opts.maxRunSeconds; end
if ~isfield(opts, 'maxConnectSeconds'), opts.maxConnectSeconds = opts.maxRunSeconds; end
if ~isfield(opts, 'maxAnimSeconds'), opts.maxAnimSeconds = opts.maxRunSeconds; end
if ~isfield(opts, 'maxConsecutiveNoProgress'), opts.maxConsecutiveNoProgress = 14; end
if ~isfield(opts, 'progressFcn'), opts.progressFcn = []; end
if ~isfield(opts, 'saveVideo'), opts.saveVideo = false; end
if ~isfield(opts, 'videoPath'), opts.videoPath = ''; end
if ~isfield(opts, 'videoFrameRate'), opts.videoFrameRate = 24; end
if ~isfield(opts, 'videoFrameStep'), opts.videoFrameStep = 1; end

ik = inverseKinematics('RigidBodyTree', robot);
nTarget = size(T_target_seq, 3);
nJ = numel(qStart);
tAll = tic;
tIk = tic;
consecutiveLocalFail = 0;
totalLocalFail = 0;
timeoutTriggered = false;
timeoutStage = 'none';
stallTriggered = false;
localRrtDisabledByFail = false;
videoSaved = false;
videoPathUsed = '';
if opts.verbose
    fprintf(['[control.run_ik] start targets=%d nJ=%d prefixPts=%d render=%d ' ...
        'tIK=%.1fs tConn=%.1fs tAnim=%.1fs\n'], ...
        nTarget, nJ, size(opts.prefixPath, 1), opts.render, ...
        opts.maxIkSeconds, opts.maxConnectSeconds, opts.maxAnimSeconds);
end
notifyProgress(opts, 'ik_start', sprintf('IK phase start (%d targets)', nTarget), 0, nTarget);

qWay = zeros(nTarget, nJ);
posErr = zeros(nTarget, 1);
oriErr = zeros(nTarget, 1);
isFallback = false(nTarget, 1);
isHold = false(nTarget, 1);
nSolved = 0;

qPrev = clamp_to_limits(qStart, jointLimits);
for i = 1:nTarget
    if toc(tIk) > opts.maxIkSeconds
        timeoutTriggered = true;
        timeoutStage = 'ik';
        if opts.verbose
            fprintf('[control.run_ik] timeout during IK target solve at %d/%d\n', i, nTarget);
        end
        notifyProgress(opts, 'timeout', sprintf('Timeout during IK solve at %d/%d', i, nTarget), i, nTarget);
        break;
    end
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
    nSolved = i;

    if opts.verbose && (i <= 5 || mod(i, 10) == 0 || i == nTarget)
        fprintf('[control.run_ik] %03d/%03d status=%s fallback=%d hold=%d posErr=%.4f oriErr=%.2f\n', ...
            i, nTarget, string(infoTry.Status), isFallback(i), isHold(i), pErr, oErr);
    end
    if mod(i, 8) == 0 || i == nTarget
        notifyProgress(opts, 'ik_solve', sprintf('IK solved %d/%d', i, nTarget), i, nTarget);
    end
end
ikElapsed = toc(tIk);

if nSolved < 1
    nSolved = 1;
    qWay(1, :) = qStart;
end

qExecuted = opts.prefixPath;
if isempty(qExecuted)
    qExecuted = qStart;
end

tConnect = tic;
consecutiveNoProgress = 0;
for i = 1:nSolved
    if toc(tConnect) > opts.maxConnectSeconds
        timeoutTriggered = true;
        timeoutStage = 'connect';
        if opts.verbose
            fprintf('[control.run_ik] timeout during connect at %d/%d\n', i, nSolved);
        end
        notifyProgress(opts, 'timeout', sprintf('Timeout during connect at %d/%d', i, nSolved), i, nSolved);
        break;
    end
    if opts.verbose && (i <= 3 || mod(i, 10) == 0 || i == nSolved)
        fprintf('[control.run_ik] connect %03d/%03d\n', i, nSolved);
    end
    qA = qExecuted(end, :);
    qB = qWay(i, :);
    [seg, segMeta] = connectSegment(qA, qB, jointLimits, collisionFcn, opts);
    segProgress = safeNumber(segMeta.segProgress, norm(qB - qA));

    if segMeta.localRrtTried
        if segMeta.localRrtSolved
            consecutiveLocalFail = 0;
        else
            consecutiveLocalFail = consecutiveLocalFail + 1;
            totalLocalFail = totalLocalFail + 1;
        end
    else
        consecutiveLocalFail = 0;
    end

    if segProgress < 1e-4
        consecutiveNoProgress = consecutiveNoProgress + 1;
    else
        consecutiveNoProgress = 0;
    end
    if opts.verbose && (segMeta.localRrtTried || segMeta.usedHoldFallback)
        fprintf(['[control.run_ik] segMeta i=%d/%d progress=%.4f localTried=%d ' ...
            'localSolved=%d holdSamples=%d noProgress=%d\n'], ...
            i, nSolved, segProgress, segMeta.localRrtTried, ...
            segMeta.localRrtSolved, segMeta.holdSamples, consecutiveNoProgress);
    end

    if opts.localRrtEnabled && (consecutiveLocalFail >= opts.maxConsecutiveLocalFail || totalLocalFail >= opts.maxTotalLocalFail)
        opts.localRrtEnabled = false;
        localRrtDisabledByFail = true;
        notifyProgress(opts, 'local_rrt_disabled', ...
            sprintf('Local RRT disabled (consecutiveFail=%d totalFail=%d)', consecutiveLocalFail, totalLocalFail), i, nSolved);
        if opts.verbose
            fprintf('[control.run_ik] disable local RRT due failures (consecutive=%d total=%d)\n', ...
                consecutiveLocalFail, totalLocalFail);
        end
    end

    if consecutiveNoProgress >= opts.maxConsecutiveNoProgress
        timeoutTriggered = true;
        timeoutStage = 'connect_stall';
        stallTriggered = true;
        if opts.verbose
            fprintf('[control.run_ik] stop connect due no-progress threshold at %d/%d\n', i, nSolved);
        end
        notifyProgress(opts, 'timeout', ...
            sprintf('Connect stalled at %d/%d (noProgress=%d)', i, nSolved, consecutiveNoProgress), i, nSolved);
        break;
    end

    if size(seg, 1) > 1
        qExecuted = [qExecuted; seg(2:end, :)]; %#ok<AGROW>
    end
    if mod(i, 8) == 0 || i == nSolved
        notifyProgress(opts, 'connect', sprintf('Connected %d/%d', i, nSolved), i, nSolved);
    end
end
connectElapsed = toc(tConnect);

tipTrail = nan(size(qExecuted, 1), 3);
animElapsed = 0;
if opts.render
    tAnim = tic;
    vw = [];
    if opts.saveVideo && ~isempty(opts.videoPath)
        try
            videoDir = fileparts(opts.videoPath);
            if ~isempty(videoDir) && ~isfolder(videoDir)
                mkdir(videoDir);
            end
            vw = VideoWriter(opts.videoPath, 'MPEG-4');
            vw.FrameRate = opts.videoFrameRate;
            open(vw);
            videoPathUsed = opts.videoPath;
            if opts.verbose
                fprintf('[control.run_ik] video record start: %s\n', opts.videoPath);
            end
        catch ME
            vw = [];
            warning('[control.run_ik] video start failed: %s', ME.message);
        end
    end

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
        if toc(tAnim) > opts.maxAnimSeconds
            timeoutTriggered = true;
            timeoutStage = 'animation';
            if opts.verbose
                fprintf('[control.run_ik] timeout during animation at %d/%d\n', i, size(qExecuted, 1));
            end
            notifyProgress(opts, 'timeout', sprintf('Timeout during animation at %d/%d', i, size(qExecuted, 1)), i, size(qExecuted, 1));
            break;
        end
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
        if ~isempty(vw) && (mod(i, max(1, opts.videoFrameStep)) == 0 || i == size(qExecuted, 1))
            try
                writeVideo(vw, getframe(ancestor(ax, 'figure')));
            catch
            end
        end
        pause(opts.framePause);
        if mod(i, 12) == 0 || i == size(qExecuted, 1)
            notifyProgress(opts, 'animate', sprintf('Animating %d/%d', i, size(qExecuted, 1)), i, size(qExecuted, 1));
        end
    end
    if ~isempty(vw)
        try
            close(vw);
            videoSaved = true;
            if opts.verbose
                fprintf('[control.run_ik] video saved: %s\n', videoPathUsed);
            end
        catch ME
            warning('[control.run_ik] video close failed: %s', ME.message);
        end
    end
    animElapsed = toc(tAnim);
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
runInfo.nSolved = nSolved;
runInfo.timeoutTriggered = timeoutTriggered;
runInfo.timeoutStage = timeoutStage;
runInfo.stallTriggered = stallTriggered;
runInfo.elapsedIkSec = ikElapsed;
runInfo.elapsedConnectSec = connectElapsed;
runInfo.elapsedAnimSec = animElapsed;
runInfo.elapsedSec = toc(tAll);
runInfo.localRrtDisabledByFail = localRrtDisabledByFail;
runInfo.localRrtFailTotal = totalLocalFail;
runInfo.videoSaved = videoSaved;
runInfo.videoPath = videoPathUsed;
notifyProgress(opts, 'done', sprintf('Run done in %.1fs (timeout=%d, stage=%s)', runInfo.elapsedSec, runInfo.timeoutTriggered, runInfo.timeoutStage), nTarget, nTarget);
if opts.verbose
    fprintf('[control.run_ik] done solved=%d/%d execPts=%d timeout=%d stage=%s t=[ik %.1fs, conn %.1fs, anim %.1fs]\n', ...
        nSolved, nTarget, runInfo.nExecuted, runInfo.timeoutTriggered, runInfo.timeoutStage, ...
        runInfo.elapsedIkSec, runInfo.elapsedConnectSec, runInfo.elapsedAnimSec);
end
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

function [qSeg, meta] = connectSegment(qA, qB, jointLimits, collisionFcn, opts)
meta = struct('localRrtTried', false, 'localRrtSolved', false, ...
    'usedHoldFallback', false, 'holdSamples', 0, 'segProgress', 0);
qSeg = linearInterpolate(qA, qB, opts.interpStepRad);
if pathCollisionFree(qSeg, collisionFcn)
    meta.segProgress = norm(qSeg(end, :) - qA);
    return;
end

if opts.localRrtEnabled
    meta.localRrtTried = true;
    if opts.verbose
        fprintf('[control.run_ik] direct segment collides, local RRT* fallback...\n');
    end
    plannerOpts = struct( ...
        'maxIter', opts.localRrtMaxIter, ...
        'goalBias', 0.25, ...
        'stepSize', 0.25, ...
        'nearRadius', 0.65, ...
        'goalThresh', 0.18, ...
        'edgeStep', opts.edgeStepRad, ...
        'shortcutIters', 50, ...
        'verbose', opts.verbose, ...
        'logEvery', 120);
    try
        [qPlan, ~] = plan_rrtstar_joint_path(qA, qB, jointLimits, collisionFcn, plannerOpts);
        if size(qPlan, 1) >= 2 && pathCollisionFree(qPlan, collisionFcn)
            qSeg = densifyPath(qPlan, opts.interpStepRad);
            meta.localRrtSolved = true;
            meta.segProgress = norm(qSeg(end, :) - qA);
            if opts.verbose
                fprintf('[control.run_ik] local RRT* solved segment, pts=%d\n', size(qSeg, 1));
            end
            return;
        end
    catch
        % Keep-going: ignore local planner error and fallback below.
        if opts.verbose
            fprintf('[control.run_ik] local RRT* error, fallback to hold interpolation.\n');
        end
    end
end

% Keep-going fallback: attempt clamped direct interpolation.
qSeg = densifyPath([qA; qB], opts.interpStepRad);
holdCount = 0;
for i = 1:size(qSeg, 1)
    if collisionFcn(qSeg(i, :))
        qSeg(i, :) = qA;
        holdCount = holdCount + 1;
    end
end
meta.usedHoldFallback = holdCount > 0;
meta.holdSamples = holdCount;
meta.segProgress = norm(qSeg(end, :) - qA);
if opts.verbose && holdCount > 0
    fprintf('[control.run_ik] hold fallback used, holdSamples=%d/%d progress=%.4f\n', ...
        holdCount, size(qSeg, 1), meta.segProgress);
end
end

function notifyProgress(opts, stage, msg, idx, total)
if isempty(opts.progressFcn)
    return;
end
try
    s = struct('stage', stage, 'message', msg, 'idx', idx, 'total', total);
    opts.progressFcn(s);
catch
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

function v = safeNumber(x, fallback)
if isempty(x) || ~isfinite(x)
    v = fallback;
else
    v = x;
end
end
