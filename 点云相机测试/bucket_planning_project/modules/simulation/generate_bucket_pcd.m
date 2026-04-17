function stats = generate_bucket_pcd(bucketParams, outPcdPath, opts)
% generate_bucket_pcd:
% Generate a thin-wall frustum bucket point cloud and optionally save to PCD.
%
% bucketParams fields:
%   topCenter [1x3], topRadius, bottomRadius, depth, wallThickness
%
% Return:
%   stats.pointCount, stats.bounds, stats.ptCloud, stats.outPcdPath

if nargin < 3
    opts = struct();
end
if nargin < 2
    outPcdPath = '';
end

validateBucketParams(bucketParams);

if ~isfield(opts, 'nSide')
    opts.nSide = 14000;
end
if ~isfield(opts, 'nBottom')
    opts.nBottom = 4000;
end
if ~isfield(opts, 'nRim')
    opts.nRim = 2500;
end
if ~isfield(opts, 'noiseSigma')
    opts.noiseSigma = 0.0015;
end
if ~isfield(opts, 'encoding') || isempty(opts.encoding)
    opts.encoding = 'ascii';
end
if ~isfield(opts, 'seed') || isempty(opts.seed)
    opts.seed = 7;
end

rng(opts.seed);

topC = bucketParams.topCenter(:)';
rTop = bucketParams.topRadius;
rBottom = bucketParams.bottomRadius;
depth = bucketParams.depth;
tWall = bucketParams.wallThickness;
tHalf = tWall / 2;

% Side surfaces
hSide = depth * rand(opts.nSide, 1);
thetaSide = 2*pi*rand(opts.nSide, 1);
rMid = rTop + (rBottom - rTop) .* (hSide ./ max(depth, 1e-9));
rOuter = rMid + tHalf;
rInner = max(rMid - tHalf, 0.001);

outerPts = [rOuter .* cos(thetaSide), rOuter .* sin(thetaSide), -hSide];
innerPts = [rInner .* cos(thetaSide), rInner .* sin(thetaSide), -hSide];

% Top rim annulus
thetaRim = 2*pi*rand(opts.nRim, 1);
rInRim = max(rTop - tHalf, 0);
rOutRim = rTop + tHalf;
rhoRim = sqrt(rand(opts.nRim, 1) .* (rOutRim^2 - rInRim^2) + rInRim^2);
rimPts = [rhoRim .* cos(thetaRim), rhoRim .* sin(thetaRim), zeros(opts.nRim, 1)];

% Bottom disk (outer cap)
thetaBottom = 2*pi*rand(opts.nBottom, 1);
rhoBottom = (rBottom + tHalf) * sqrt(rand(opts.nBottom, 1));
bottomPts = [rhoBottom .* cos(thetaBottom), rhoBottom .* sin(thetaBottom), -depth * ones(opts.nBottom, 1)];

xyz = [outerPts; innerPts; rimPts; bottomPts];
xyz(:, 1:2) = xyz(:, 1:2) + topC(1:2);
xyz(:, 3) = xyz(:, 3) + topC(3);

if opts.noiseSigma > 0
    xyz = xyz + opts.noiseSigma * randn(size(xyz));
end

ptCloud = pointCloud(xyz);

if ~isempty(outPcdPath)
    outDir = fileparts(outPcdPath);
    if ~isempty(outDir) && ~isfolder(outDir)
        mkdir(outDir);
    end
    pcwrite(ptCloud, outPcdPath, 'Encoding', opts.encoding);
    fprintf('[simulation.generate_bucket_pcd] Saved PCD: %s (points=%d)\n', outPcdPath, ptCloud.Count);
end

stats = struct();
stats.pointCount = ptCloud.Count;
stats.bounds = [min(xyz, [], 1); max(xyz, [], 1)];
stats.ptCloud = ptCloud;
stats.outPcdPath = outPcdPath;
stats.noiseSigma = opts.noiseSigma;
stats.bucketParams = bucketParams;
end

function validateBucketParams(p)
required = {'topCenter', 'topRadius', 'bottomRadius', 'depth', 'wallThickness'};
for i = 1:numel(required)
    if ~isfield(p, required{i})
        error('[simulation.generate_bucket_pcd] Missing bucketParams.%s', required{i});
    end
end

if numel(p.topCenter) ~= 3
    error('[simulation.generate_bucket_pcd] topCenter must be [1x3].');
end
if p.topRadius <= 0 || p.bottomRadius <= 0 || p.depth <= 0 || p.wallThickness <= 0
    error('[simulation.generate_bucket_pcd] radius/depth/wallThickness must be positive.');
end
if p.wallThickness >= min(p.topRadius, p.bottomRadius)
    error('[simulation.generate_bucket_pcd] wallThickness must be smaller than bucket radii.');
end
end
