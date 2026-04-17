function [digDirection, pcaInfo] = extract_bucket_axis_pca(pointsXYZ)
% extract_bucket_axis_pca:
% Estimate horizontal digging direction from point cloud distribution.

if isempty(pointsXYZ) || size(pointsXYZ, 2) ~= 3
    error('[perception.extract_bucket_axis_pca] pointsXYZ must be Nx3.');
end

[coeff, ~, latent] = pca(pointsXYZ);
mainAxis = coeff(:, 1);
h = [mainAxis(1), mainAxis(2), 0];
if norm(h) < 1e-9
    h = [1, 0, 0];
end
h = h / norm(h);

% Keep deterministic direction sign.
if h(1) < 0
    h = -h;
end

digDirection = h;
pcaInfo = struct('coeff', coeff, 'latent', latent);
end

