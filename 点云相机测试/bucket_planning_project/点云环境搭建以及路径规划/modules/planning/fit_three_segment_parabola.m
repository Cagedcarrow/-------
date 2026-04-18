function [pathBase, segments, keyPts, T_target_seq, fitInfo] = ...
    fit_three_segment_parabola(bucketFeatures, trajParams, postureParams, nPts)
% fit_three_segment_parabola:
% Fit a three-segment adjustable parabola in the bucket local X-Z plane.
% Segment order: steep-in -> gentle-middle -> steep-out.

if nargin < 4 || isempty(nPts)
    nPts = 90;
end
if nargin < 3
    postureParams = struct();
end
if nargin < 2
    trajParams = struct();
end

trajParams = fillDefaultTrajParams(trajParams);
if ~isfield(postureParams, 'attackDeg'), postureParams.attackDeg = 0; end
if ~isfield(postureParams, 'assemblyDeg'), postureParams.assemblyDeg = 0; end
if ~isfield(postureParams, 'flipToolZ'), postureParams.flipToolZ = true; end

topPoint = bucketFeatures.topPoint(:)';
maxR = bucketFeatures.maxRadius;
depth = bucketFeatures.depth;
if isfield(bucketFeatures, 'topRadius') && ~isempty(bucketFeatures.topRadius)
    topR = bucketFeatures.topRadius;
else
    topR = maxR;
end

if trajParams.forceBaseXDir
    xAxis = [1; 0; 0];
    xAxisSource = 'base_x';
else
    xAxis = bucketFeatures.digDirection(:);
    xAxis(3) = 0;
    if norm(xAxis) < 1e-8
        xAxis = [1; 0; 0];
    end
    xAxis = xAxis / norm(xAxis);
    xAxisSource = 'perception_pca';
end
if isfield(trajParams, 'reverseFitZAxis') && trajParams.reverseFitZAxis
    zAxis = [0; 0; -1];
    zAxisSource = 'reverse';
else
    zAxis = [0; 0; 1];
    zAxisSource = 'normal';
end
yAxis = cross(zAxis, xAxis);
yAxis = yAxis / max(norm(yAxis), 1e-12);

% Key points in local coordinates.
outerOffset = trajParams.topOuterOffsetRatio * topR;
startFinishRadius = max(0.02, topR + outerOffset); % outside bucket top radius
x0 = -trajParams.entrySpanFactor * startFinishRadius;
x1 = -trajParams.midSpanFactor * maxR;
x2 = 0;
x3 = trajParams.exitSpanFactor * startFinishRadius;
if x1 <= x0
    x1 = 0.5 * (x0 + x2);
end

zTopLift = topPoint(3) + trajParams.startFinishLiftRatio * depth;
z0 = zTopLift;
z1 = topPoint(3) - trajParams.entryDepthRatio * depth;
z2 = topPoint(3) - trajParams.targetDeepDepthRatio * depth;
z3 = zTopLift;

lsInfo = tryLeastSquareProfile(bucketFeatures, xAxis, yAxis, topPoint, [x0, x1, x2, x3], [z0, z1, z2, z3]);
if lsInfo.valid
    z1 = lsInfo.zKey(2);
end
% Keep start/finish anchors fixed by construction.
z0 = zTopLift;
z3 = zTopLift;
z2 = topPoint(3) - trajParams.targetDeepDepthRatio * depth;

% Safety envelope:
% 1) Deep point is fixed to targetDeepDepthRatio * bucketDepth.
% 2) Entry/exit slope should not exceed maxCutAngleDeg.
zDeepTarget = topPoint(3) - trajParams.targetDeepDepthRatio * depth;
z2 = zDeepTarget;
z1 = max(z1, z2 + trajParams.minDeltaAroundDeep);
z1 = min(z1, z0 - 0.02);
z3 = zTopLift;

% Segment 1 and 2 are constrained by slope at x1 (gentle middle entry).
maxSlope = tand(trajParams.maxCutAngleDeg);
safeSlope1 = clampSlope(trajParams.midSlopeAtKnot1, maxSlope);
if trajParams.execFirstMode
    % Ensure x1->x2 chord does not force over-steep descent.
    z1MaxBySlope = z2 + maxSlope * abs(x2 - x1);
    z1 = min(z1, z1MaxBySlope);
end
seg1 = solveQuadWithEndSlope(x0, z0, x1, z1, safeSlope1);
if trajParams.execFirstMode
    % Execution priority: make middle segment linear to avoid steep rebound near deep knot.
    seg2 = solveLineBetweenPoints(x1, z1, x2, z2);
else
    seg2 = solveQuadWithStartSlope(x1, z1, x2, z2, safeSlope1);
end
sKnot2 = 2 * seg2.a * x2 + seg2.b;
safeSlope2 = clampSlope(sKnot2, maxSlope);
if trajParams.execFirstMode
    % Execution priority: relax C1 at deep point and align exit segment with
    % endpoint chord to avoid over-steep rebound and preserve deep target.
    chordSlope23 = (z3 - z2) / max((x3 - x2), 1e-8);
    safeSlope3 = clampSlope(chordSlope23, maxSlope);
else
    safeSlope3 = safeSlope2;
end
seg3 = solveQuadWithStartSlope(x2, z2, x3, z3, safeSlope3);

n1 = max(5, round(nPts * trajParams.nRatio(1)));
n2 = max(5, round(nPts * trajParams.nRatio(2)));
n3 = max(5, nPts - n1 - n2 + 2);

xSeg1 = linspace(x0, x1, n1)';
xSeg2 = linspace(x1, x2, n2)';
xSeg3 = linspace(x2, x3, n3)';

zSeg1 = evalQuad(seg1, xSeg1);
zSeg2 = evalQuad(seg2, xSeg2);
zSeg3 = evalQuad(seg3, xSeg3);
if trajParams.execFirstMode
    % Keep deep point around requested depth and avoid undershoot below target.
    zSeg2 = max(zSeg2, z2);
    zSeg3 = max(zSeg3, z2);
end

% Remove duplicated knots while concatenating.
xLocal = [xSeg1; xSeg2(2:end); xSeg3(2:end)];
zLocal = [zSeg1; zSeg2(2:end); zSeg3(2:end)];
yLocal = trajParams.localYOffset * ones(size(xLocal));
[zLocal, scaleUsed, maxSlopeAfter] = enforceSlopeWithExecPriority( ...
    xLocal, zLocal, maxSlope, z0, z3, trajParams.execFirstMode, z2);

pathBase = localToWorld(topPoint, xAxis, yAxis, xLocal, yLocal, zLocal);

keyPts = struct();
zStartNow = zLocal(1);
[~, idDeep] = min(zLocal);
zDeepNow = zLocal(idDeep);
zFinishNow = zLocal(end);
zKnot1Now = interp1(xLocal, zLocal, x1, 'linear', 'extrap');
keyPts.start = localToWorld(topPoint, xAxis, yAxis, x0, trajParams.localYOffset, zStartNow);
keyPts.knot1 = localToWorld(topPoint, xAxis, yAxis, x1, trajParams.localYOffset, zKnot1Now);
keyPts.deep = localToWorld(topPoint, xAxis, yAxis, xLocal(idDeep), trajParams.localYOffset, zDeepNow);
keyPts.finish = localToWorld(topPoint, xAxis, yAxis, x3, trajParams.localYOffset, zFinishNow);

segments = struct();
segments.seg1 = seg1;
segments.seg2 = seg2;
segments.seg3 = seg3;
segments.knotSlope1 = trajParams.midSlopeAtKnot1;
segments.knotSlope1Used = safeSlope1;
segments.knotSlope2 = sKnot2;
segments.knotSlope2Used = safeSlope2;
segments.knotSlope3Used = safeSlope3;
segments.localX = xLocal;
segments.localZ = zLocal;

[T_target_seq, poseInfo] = build_target_pose_seq(pathBase, postureParams, struct('flipToolZ', postureParams.flipToolZ));

fitInfo = struct();
fitInfo.localFrame = struct('xAxis', xAxis, 'yAxis', yAxis, 'zAxis', zAxis, 'originTop', topPoint);
fitInfo.lsInfo = lsInfo;
fitInfo.poseInfo = poseInfo;
fitInfo.numPoints = size(pathBase, 1);
fitInfo.keyX = [x0, x1, x2, x3];
fitInfo.keyZ = [zStartNow, zKnot1Now, zDeepNow, zFinishNow];
fitInfo.rmse = lsInfo.rmse;
fitInfo.slopeMaxAllowed = maxSlope;
fitInfo.slopeMaxAfter = maxSlopeAfter;
fitInfo.slopeScaleUsed = scaleUsed;
fitInfo.deepTargetZ = zDeepTarget;
fitInfo.deepAbs = topPoint(3) - zDeepNow;
fitInfo.deepTargetAbs = topPoint(3) - zDeepTarget;
fitInfo.startFinishRadius = startFinishRadius;
fitInfo.outerOffset = outerOffset;
fitInfo.startFinishLiftZ = zTopLift;
fitInfo.execFirstMode = trajParams.execFirstMode;
fitInfo.xAxisSource = xAxisSource;
fitInfo.zAxisSource = zAxisSource;

fprintf(['[planning.fit_three_segment_parabola] points=%d, rmse=%.5f, ' ...
    'zKey=[%.3f %.3f %.3f %.3f], deep=%.3f/%.3f, slopeUsed=[%.3f %.3f %.3f], maxSlopeAfter=%.3f, scale=%.3f, xAxis=%s zAxis=%s\n'], ...
    fitInfo.numPoints, fitInfo.rmse, zStartNow, zKnot1Now, zDeepNow, zFinishNow, ...
    fitInfo.deepAbs, fitInfo.deepTargetAbs, ...
    segments.knotSlope1Used, segments.knotSlope2Used, segments.knotSlope3Used, ...
    fitInfo.slopeMaxAfter, fitInfo.slopeScaleUsed, fitInfo.xAxisSource, fitInfo.zAxisSource);
if fitInfo.slopeMaxAfter > (fitInfo.slopeMaxAllowed + 1e-6)
    fprintf('[planning.fit_three_segment_parabola][WARN] slope %.3f exceeds allowed %.3f\n', ...
        fitInfo.slopeMaxAfter, fitInfo.slopeMaxAllowed);
elseif fitInfo.slopeScaleUsed < 0.999
    fprintf('[planning.fit_three_segment_parabola][INFO] slope scaled for execution, scale=%.3f\n', ...
        fitInfo.slopeScaleUsed);
end
if fitInfo.deepAbs < 0
    fprintf('[planning.fit_three_segment_parabola][WARN] deep point above top plane: deepAbs=%.3f\n', fitInfo.deepAbs);
elseif abs(fitInfo.deepAbs - fitInfo.deepTargetAbs) > 0.03
    fprintf('[planning.fit_three_segment_parabola][INFO] deep mismatch %.3f (target %.3f)\n', ...
        fitInfo.deepAbs, fitInfo.deepTargetAbs);
end
end

function trajParams = fillDefaultTrajParams(trajParams)
if ~isfield(trajParams, 'entrySpanFactor'), trajParams.entrySpanFactor = 1.00; end
if ~isfield(trajParams, 'midSpanFactor'), trajParams.midSpanFactor = 0.35; end
if ~isfield(trajParams, 'exitSpanFactor'), trajParams.exitSpanFactor = 1.00; end
if ~isfield(trajParams, 'entryLift'), trajParams.entryLift = 0.04; end
if ~isfield(trajParams, 'entryDepthRatio'), trajParams.entryDepthRatio = 0.45; end
if ~isfield(trajParams, 'deepDepthRatio'), trajParams.deepDepthRatio = 0.88; end
if ~isfield(trajParams, 'exitDepthRatio'), trajParams.exitDepthRatio = 0.22; end
if ~isfield(trajParams, 'midSlopeAtKnot1'), trajParams.midSlopeAtKnot1 = -0.20; end
if ~isfield(trajParams, 'localYOffset'), trajParams.localYOffset = 0.0; end
if ~isfield(trajParams, 'nRatio'), trajParams.nRatio = [0.36, 0.28, 0.36]; end
if ~isfield(trajParams, 'maxCutAngleDeg'), trajParams.maxCutAngleDeg = 30.0; end
if ~isfield(trajParams, 'maxDeepDepthRatio'), trajParams.maxDeepDepthRatio = 0.25; end
if ~isfield(trajParams, 'minDeltaAroundDeep'), trajParams.minDeltaAroundDeep = 0.04; end
if ~isfield(trajParams, 'topOuterOffsetRatio'), trajParams.topOuterOffsetRatio = 0.125; end
if ~isfield(trajParams, 'startFinishLiftRatio'), trajParams.startFinishLiftRatio = 0.25; end
if ~isfield(trajParams, 'targetDeepDepthRatio'), trajParams.targetDeepDepthRatio = 0.15; end
if ~isfield(trajParams, 'execFirstMode'), trajParams.execFirstMode = true; end
if ~isfield(trajParams, 'forceBaseXDir'), trajParams.forceBaseXDir = true; end
if ~isfield(trajParams, 'reverseFitZAxis'), trajParams.reverseFitZAxis = false; end
end

function s = clampSlope(sIn, sMax)
s = min(max(sIn, -abs(sMax)), abs(sMax));
end

function seg = solveQuadWithEndSlope(xa, za, xb, zb, slopeB)
A = [xa^2, xa, 1; xb^2, xb, 1; 2*xb, 1, 0];
b = [za; zb; slopeB];
abc = A \ b;
seg = struct('a', abc(1), 'b', abc(2), 'c', abc(3), 'xRange', [xa, xb]);
end

function seg = solveQuadWithStartSlope(xa, za, xb, zb, slopeA)
A = [xa^2, xa, 1; xb^2, xb, 1; 2*xa, 1, 0];
b = [za; zb; slopeA];
abc = A \ b;
seg = struct('a', abc(1), 'b', abc(2), 'c', abc(3), 'xRange', [xa, xb]);
end

function seg = solveLineBetweenPoints(xa, za, xb, zb)
k = (zb - za) / max((xb - xa), 1e-8);
c = za - k * xa;
seg = struct('a', 0.0, 'b', k, 'c', c, 'xRange', [xa, xb]);
end

function z = evalQuad(seg, x)
z = seg.a * x.^2 + seg.b * x + seg.c;
end

function p = localToWorld(topPoint, xAxis, yAxis, x, y, zAbs)
x = x(:);
y = y(:);
zAbs = zAbs(:);
n = numel(x);
rel = x .* xAxis' + y .* yAxis' + (zAbs - topPoint(3)) .* [0, 0, 1];
p = rel + repmat(topPoint, n, 1);
if n == 1
    p = p(1, :);
end
end

function lsInfo = tryLeastSquareProfile(bucketFeatures, xAxis, yAxis, topPoint, keyX, keyZ)
lsInfo = struct('valid', false, 'rmse', nan, 'abc', [nan, nan, nan], 'zKey', keyZ);
if ~isfield(bucketFeatures, 'cloudPoints') || isempty(bucketFeatures.cloudPoints)
    return;
end

pts = bucketFeatures.cloudPoints;
rel = pts - topPoint;
xProj = rel * xAxis(:);
yProj = rel * yAxis(:);
zAbs = pts(:, 3);

band = max(0.02, 0.12 * bucketFeatures.maxRadius);
mask = abs(yProj) <= band;
if sum(mask) < 120
    return;
end

xFit = xProj(mask);
zFit = zAbs(mask);
A = [xFit.^2, xFit, ones(size(xFit))];
abc = A \ zFit;
zPred = A * abc;
rmse = sqrt(mean((zPred - zFit).^2));

zKeyLS = abc(1) * keyX.^2 + abc(2) * keyX + abc(3);
zKeyBlend = 0.50 * keyZ + 0.50 * zKeyLS;

% Keep depth ordering.
zTop = topPoint(3);
zKeyBlend(1) = min(max(zKeyBlend(1), zTop - 0.02), zTop + 0.10);
zKeyBlend(2) = min(zKeyBlend(2), zTop - 0.08);
zKeyBlend(3) = min(zKeyBlend(3), zTop - 0.18);
zKeyBlend(4) = min(zKeyBlend(4), zTop - 0.04);
zKeyBlend(3) = min(zKeyBlend(3), zKeyBlend(2) - 0.03);
zKeyBlend(4) = max(zKeyBlend(4), zKeyBlend(3) + 0.06);

lsInfo.valid = true;
lsInfo.rmse = rmse;
lsInfo.abc = abc';
lsInfo.zKey = zKeyBlend;
end

function maxSlopeAfter = calcMaxSlope(xIn, zIn)
dx = diff(xIn);
dz = diff(zIn);
s = abs(dz ./ max(abs(dx), 1e-8));
maxSlopeAfter = max(s);
end

function [zOut, scaleUsed, maxSlopeAfter] = enforceSlopeWithExecPriority( ...
    xIn, zIn, maxSlope, zStart, zFinish, execFirstMode, zDeepTarget)
zOut = zIn;
zOut(1) = zStart;
zOut(end) = zFinish;
scaleUsed = 1.0;
if nargin < 7
    zDeepTarget = NaN;
end

if ~execFirstMode
    maxSlopeAfter = calcMaxSlope(xIn, zOut);
    return;
end

idDeep = 1;
if ~isempty(xIn)
    [~, idDeep] = min(abs(xIn));
end
if isfinite(zDeepTarget)
    zOut(idDeep) = zDeepTarget;
end

for k = 1:8
    zPrev = zOut;
    for i = 2:numel(zOut)
        maxDz = maxSlope * abs(xIn(i) - xIn(i - 1));
        dz = zOut(i) - zOut(i - 1);
        if dz > maxDz
            zOut(i) = zOut(i - 1) + maxDz;
        elseif dz < -maxDz
            zOut(i) = zOut(i - 1) - maxDz;
        end
    end
    for i = (numel(zOut) - 1):-1:1
        maxDz = maxSlope * abs(xIn(i + 1) - xIn(i));
        dz = zOut(i) - zOut(i + 1);
        if dz > maxDz
            zOut(i) = zOut(i + 1) + maxDz;
        elseif dz < -maxDz
            zOut(i) = zOut(i + 1) - maxDz;
        end
    end
    zOut(1) = zStart;
    zOut(end) = zFinish;
    if isfinite(zDeepTarget)
        zOut(idDeep) = zDeepTarget;
    end
    if max(abs(zOut - zPrev)) < 1e-6
        break;
    end
end

maxSlopeAfter = calcMaxSlope(xIn, zOut);
zRef = 0.5 * (zStart + zFinish);
ampIn = max(abs(zIn - zRef));
ampOut = max(abs(zOut - zRef));
if ampIn > 1e-9
    scaleUsed = ampOut / ampIn;
else
    scaleUsed = 1.0;
end
end
