function result = estimate_bucket_from_pcd(pcdPath, opts)
% estimate_bucket_from_pcd:
% Read bucket PCD and estimate geometric features.
%
% Return fields:
%   topPoint, bottomRadius, depth, maxRadius, digDirection, debugInfo

if nargin < 2
    opts = struct();
end
if ~isfield(opts, 'gridStep')
    opts.gridStep = 0.006;
end
if ~isfield(opts, 'topSliceRatio')
    opts.topSliceRatio = 0.10;
end
if ~isfield(opts, 'bottomSliceRatio')
    opts.bottomSliceRatio = 0.12;
end
if ~isfield(opts, 'centerMethod')
    opts.centerMethod = 'median';
end

fprintf('[perception.estimate_bucket_from_pcd] Loading: %s\n', pcdPath);
if ~isfile(pcdPath)
    error('[perception.estimate_bucket_from_pcd] File not found: %s', pcdPath);
end

ptCloudRaw = pcread(pcdPath);
if ptCloudRaw.Count < 100
    error('[perception.estimate_bucket_from_pcd] Too few points: %d', ptCloudRaw.Count);
end

ptCloud = pcdownsample(ptCloudRaw, 'gridAverage', opts.gridStep);
xyz = ptCloud.Location;

zMax = max(xyz(:, 3));
zMin = min(xyz(:, 3));
depth = zMax - zMin;
if depth < 1e-4
    error('[perception.estimate_bucket_from_pcd] Estimated depth too small.');
end

topBand = zMax - opts.topSliceRatio * depth;
bottomBand = zMin + opts.bottomSliceRatio * depth;

topPts = xyz(xyz(:, 3) >= topBand, :);
bottomPts = xyz(xyz(:, 3) <= bottomBand, :);

if size(topPts, 1) < 40 || size(bottomPts, 1) < 40
    error('[perception.estimate_bucket_from_pcd] Not enough top/bottom slice points.');
end

switch lower(opts.centerMethod)
    case 'mean'
        centerXY = mean(topPts(:, 1:2), 1);
    otherwise
        centerXY = median(topPts(:, 1:2), 1);
end

topPoint = [centerXY, zMax];
rhoAll = hypot(xyz(:, 1) - centerXY(1), xyz(:, 2) - centerXY(2));
rhoTop = hypot(topPts(:, 1) - centerXY(1), topPts(:, 2) - centerXY(2));
rhoBottom = hypot(bottomPts(:, 1) - centerXY(1), bottomPts(:, 2) - centerXY(2));

topRadius = quantile(rhoTop, 0.95);
bottomRadius = quantile(rhoBottom, 0.95);
maxRadius = quantile(rhoAll, 0.98);

[digDirection, pcaInfo] = extract_bucket_axis_pca(xyz);

result = struct();
result.topPoint = topPoint;
result.topRadius = topRadius;
result.bottomRadius = bottomRadius;
result.depth = depth;
result.maxRadius = maxRadius;
result.digDirection = digDirection;
result.cloudPoints = xyz;
result.debugInfo = struct();
result.debugInfo.rawCount = ptCloudRaw.Count;
result.debugInfo.downsampledCount = ptCloud.Count;
result.debugInfo.zMax = zMax;
result.debugInfo.zMin = zMin;
result.debugInfo.topSliceCount = size(topPts, 1);
result.debugInfo.bottomSliceCount = size(bottomPts, 1);
result.debugInfo.centerXY = centerXY;
result.debugInfo.pcaLatent = pcaInfo.latent;
result.debugInfo.bounds = [min(xyz, [], 1); max(xyz, [], 1)];

fprintf(['[perception.estimate_bucket_from_pcd] top=[%.3f %.3f %.3f], ' ...
    'rTop=%.3f, rBottom=%.3f, depth=%.3f, maxR=%.3f, dir=[%.3f %.3f %.3f]\n'], ...
    result.topPoint(1), result.topPoint(2), result.topPoint(3), ...
    result.topRadius, result.bottomRadius, result.depth, result.maxRadius, ...
    result.digDirection(1), result.digDirection(2), result.digDirection(3));
end

