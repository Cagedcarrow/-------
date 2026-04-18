function out = build_bucket_stl_from_pcd(pcdPath, stlPath, opts)
% build_bucket_stl_from_pcd:
% Estimate bucket geometry from PCD and build a triangulated STL mesh.
%
% Inputs:
%   pcdPath : source point cloud path
%   stlPath : output STL path (empty => do not save)
%   opts    : optional fields
%       .features   : precomputed bucket features (skip re-estimation)
%       .gridStep   : perception downsample grid (default 0.006)
%       .nTheta     : circumferential samples (default 96)
%       .nHeight    : vertical samples (default 40)
%       .verbose    : print logs (default true)
%
% Output:
%   out.features / out.vertices / out.faces / out.stlPath / out.pointCount

if nargin < 3
    opts = struct();
end
if ~isfield(opts, 'gridStep'), opts.gridStep = 0.006; end
if ~isfield(opts, 'nTheta'), opts.nTheta = 96; end
if ~isfield(opts, 'nHeight'), opts.nHeight = 40; end
if ~isfield(opts, 'verbose'), opts.verbose = true; end

if opts.verbose
    fprintf('[simulation.build_stl] pcd=%s\n', pcdPath);
end

if isfield(opts, 'features') && ~isempty(opts.features)
    features = opts.features;
else
    features = estimate_bucket_from_pcd(pcdPath, struct('gridStep', opts.gridStep));
end

if ~isfield(features, 'topPoint') || ~isfield(features, 'topRadius') || ...
        ~isfield(features, 'bottomRadius') || ~isfield(features, 'depth')
    error('[simulation.build_stl] invalid features input.');
end

[V, F] = buildFrustumMesh(features.topPoint, features.topRadius, ...
    features.bottomRadius, features.depth, opts.nTheta, opts.nHeight);

stlSaved = false;
if nargin >= 2 && ~isempty(stlPath)
    outDir = fileparts(stlPath);
    if ~isempty(outDir) && ~isfolder(outDir)
        mkdir(outDir);
    end
    triObj = triangulation(F, V);
    stlSaved = tryWriteStl(stlPath, triObj, F, V);
end

out = struct();
out.features = features;
out.vertices = V;
out.faces = F;
out.stlPath = stlPath;
out.stlSaved = stlSaved;
out.pointCount = size(V, 1);
out.faceCount = size(F, 1);

if opts.verbose
    fprintf('[simulation.build_stl] vertices=%d faces=%d saved=%d\n', ...
        out.pointCount, out.faceCount, out.stlSaved);
end
end

function [V, F] = buildFrustumMesh(topCenter, topR, bottomR, depth, nTheta, nHeight)
nTheta = max(24, round(nTheta));
nHeight = max(8, round(nHeight));
theta = linspace(0, 2 * pi, nTheta + 1);
theta(end) = [];
hList = linspace(0, depth, nHeight);

V = zeros(nTheta * nHeight + 1, 3);
idx = @(ih, it) (ih - 1) * nTheta + it;

for ih = 1:nHeight
    h = hList(ih);
    z = topCenter(3) - h;
    r = topR + (bottomR - topR) * (h / max(depth, 1e-9));
    for it = 1:nTheta
        id = idx(ih, it);
        V(id, :) = [ ...
            topCenter(1) + r * cos(theta(it)), ...
            topCenter(2) + r * sin(theta(it)), ...
            z];
    end
end

bottomCenterId = size(V, 1);
V(bottomCenterId, :) = [topCenter(1), topCenter(2), topCenter(3) - depth];

F = zeros((nHeight - 1) * nTheta * 2 + nTheta, 3);
fId = 0;

for ih = 1:(nHeight - 1)
    for it = 1:nTheta
        it2 = it + 1;
        if it2 > nTheta
            it2 = 1;
        end

        a = idx(ih, it);
        b = idx(ih, it2);
        c = idx(ih + 1, it);
        d = idx(ih + 1, it2);

        fId = fId + 1;
        F(fId, :) = [a, c, b];
        fId = fId + 1;
        F(fId, :) = [b, c, d];
    end
end

for it = 1:nTheta
    it2 = it + 1;
    if it2 > nTheta
        it2 = 1;
    end
    a = idx(nHeight, it);
    b = idx(nHeight, it2);
    fId = fId + 1;
    F(fId, :) = [bottomCenterId, b, a];
end
end

function ok = tryWriteStl(stlPath, triObj, F, V)
ok = false;

try
    stlwrite(triObj, stlPath);
    ok = true;
    return;
catch
end

try
    stlwrite(stlPath, triObj);
    ok = true;
    return;
catch
end

try
    writeAsciiStl(stlPath, F, V);
    ok = true;
catch ME
    warning('[simulation.build_stl] STL write failed: %s', ME.message);
end
end

function writeAsciiStl(stlPath, F, V)
fid = fopen(stlPath, 'w');
if fid < 0
    error('open failed: %s', stlPath);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, 'solid bucket\n');
for i = 1:size(F, 1)
    id = F(i, :);
    p1 = V(id(1), :);
    p2 = V(id(2), :);
    p3 = V(id(3), :);
    n = cross(p2 - p1, p3 - p1);
    nn = norm(n);
    if nn > 1e-12
        n = n / nn;
    else
        n = [0, 0, 1];
    end
    fprintf(fid, '  facet normal %.7g %.7g %.7g\n', n(1), n(2), n(3));
    fprintf(fid, '    outer loop\n');
    fprintf(fid, '      vertex %.7g %.7g %.7g\n', p1(1), p1(2), p1(3));
    fprintf(fid, '      vertex %.7g %.7g %.7g\n', p2(1), p2(2), p2(3));
    fprintf(fid, '      vertex %.7g %.7g %.7g\n', p3(1), p3(2), p3(3));
    fprintf(fid, '    endloop\n');
    fprintf(fid, '  endfacet\n');
end
fprintf(fid, 'endsolid bucket\n');
end
