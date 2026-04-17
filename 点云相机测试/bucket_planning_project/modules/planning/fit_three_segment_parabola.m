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

xAxis = bucketFeatures.digDirection(:);
xAxis(3) = 0;
if norm(xAxis) < 1e-8
    xAxis = [1; 0; 0];
end
xAxis = xAxis / norm(xAxis);
zAxis = [0; 0; 1];
yAxis = cross(zAxis, xAxis);
yAxis = yAxis / max(norm(yAxis), 1e-12);

% Key points in local coordinates.
x0 = -trajParams.entrySpanFactor * maxR;
x1 = -trajParams.midSpanFactor * maxR;
x2 = 0;
x3 = trajParams.exitSpanFactor * maxR;

z0 = topPoint(3) + trajParams.entryLift;
z1 = topPoint(3) - trajParams.entryDepthRatio * depth;
z2 = topPoint(3) - trajParams.deepDepthRatio * depth;
z3 = topPoint(3) - trajParams.exitDepthRatio * depth;

lsInfo = tryLeastSquareProfile(bucketFeatures, xAxis, yAxis, topPoint, [x0, x1, x2, x3], [z0, z1, z2, z3]);
if lsInfo.valid
    z1 = lsInfo.zKey(2);
    z2 = lsInfo.zKey(3);
    z3 = lsInfo.zKey(4);
end

% Segment 1 and 2 are constrained by slope at x1 (gentle middle entry).
seg1 = solveQuadWithEndSlope(x0, z0, x1, z1, trajParams.midSlopeAtKnot1);
seg2 = solveQuadWithStartSlope(x1, z1, x2, z2, trajParams.midSlopeAtKnot1);
sKnot2 = 2 * seg2.a * x2 + seg2.b;
seg3 = solveQuadWithStartSlope(x2, z2, x3, z3, sKnot2);

n1 = max(5, round(nPts * trajParams.nRatio(1)));
n2 = max(5, round(nPts * trajParams.nRatio(2)));
n3 = max(5, nPts - n1 - n2 + 2);

xSeg1 = linspace(x0, x1, n1)';
xSeg2 = linspace(x1, x2, n2)';
xSeg3 = linspace(x2, x3, n3)';

zSeg1 = evalQuad(seg1, xSeg1);
zSeg2 = evalQuad(seg2, xSeg2);
zSeg3 = evalQuad(seg3, xSeg3);

% Remove duplicated knots while concatenating.
xLocal = [xSeg1; xSeg2(2:end); xSeg3(2:end)];
zLocal = [zSeg1; zSeg2(2:end); zSeg3(2:end)];
yLocal = trajParams.localYOffset * ones(size(xLocal));

pathBase = localToWorld(topPoint, xAxis, yAxis, xLocal, yLocal, zLocal);

keyPts = struct();
keyPts.start = localToWorld(topPoint, xAxis, yAxis, x0, trajParams.localYOffset, z0);
keyPts.knot1 = localToWorld(topPoint, xAxis, yAxis, x1, trajParams.localYOffset, z1);
keyPts.deep = localToWorld(topPoint, xAxis, yAxis, x2, trajParams.localYOffset, z2);
keyPts.finish = localToWorld(topPoint, xAxis, yAxis, x3, trajParams.localYOffset, z3);

segments = struct();
segments.seg1 = seg1;
segments.seg2 = seg2;
segments.seg3 = seg3;
segments.knotSlope1 = trajParams.midSlopeAtKnot1;
segments.knotSlope2 = sKnot2;
segments.localX = xLocal;
segments.localZ = zLocal;

[T_target_seq, poseInfo] = build_target_pose_seq(pathBase, postureParams, struct('flipToolZ', postureParams.flipToolZ));

fitInfo = struct();
fitInfo.localFrame = struct('xAxis', xAxis, 'yAxis', yAxis, 'zAxis', zAxis, 'originTop', topPoint);
fitInfo.lsInfo = lsInfo;
fitInfo.poseInfo = poseInfo;
fitInfo.numPoints = size(pathBase, 1);
fitInfo.keyX = [x0, x1, x2, x3];
fitInfo.keyZ = [z0, z1, z2, z3];
fitInfo.rmse = lsInfo.rmse;

fprintf(['[planning.fit_three_segment_parabola] points=%d, rmse=%.5f, ' ...
    'zKey=[%.3f %.3f %.3f %.3f], slopeKnot1=%.3f, slopeKnot2=%.3f\n'], ...
    fitInfo.numPoints, fitInfo.rmse, z0, z1, z2, z3, segments.knotSlope1, segments.knotSlope2);
end

function trajParams = fillDefaultTrajParams(trajParams)
if ~isfield(trajParams, 'entrySpanFactor'), trajParams.entrySpanFactor = 0.95; end
if ~isfield(trajParams, 'midSpanFactor'), trajParams.midSpanFactor = 0.35; end
if ~isfield(trajParams, 'exitSpanFactor'), trajParams.exitSpanFactor = 0.90; end
if ~isfield(trajParams, 'entryLift'), trajParams.entryLift = 0.04; end
if ~isfield(trajParams, 'entryDepthRatio'), trajParams.entryDepthRatio = 0.45; end
if ~isfield(trajParams, 'deepDepthRatio'), trajParams.deepDepthRatio = 0.88; end
if ~isfield(trajParams, 'exitDepthRatio'), trajParams.exitDepthRatio = 0.22; end
if ~isfield(trajParams, 'midSlopeAtKnot1'), trajParams.midSlopeAtKnot1 = -0.20; end
if ~isfield(trajParams, 'localYOffset'), trajParams.localYOffset = 0.0; end
if ~isfield(trajParams, 'nRatio'), trajParams.nRatio = [0.36, 0.28, 0.36]; end
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
