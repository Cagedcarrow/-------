perceive_visualize_scene_main();

function perceive_visualize_scene_main()
clc;
close all;

thisDir = fileparts(mfilename('fullpath'));
commonDir = fullfile(fileparts(thisDir), 'modules', 'common');
if exist('setup_project_paths', 'file') ~= 2
    addpath(commonDir);
end
ctx = setup_project_paths();
pcdPath = fullfile(ctx.dataDir, 'bucket_scene.pcd');
cfgPath = fullfile(ctx.dataDir, 'scene_config.mat');
outPath = fullfile(ctx.dataDir, 'perception_result.mat');

if ~isfile(pcdPath)
    error('[main.02] Missing file: %s. Run main/app1_environment_setup.m first.', pcdPath);
end

if isfile(cfgPath)
    sceneCfg = load(cfgPath);
else
    warning('[main.02] scene_config.mat not found. Use fallback defaults.');
    sceneCfg = struct();
end

features = estimate_bucket_from_pcd(pcdPath, struct('gridStep', 0.006));
ptCloud = pcread(pcdPath);

if isfield(sceneCfg, 'bucketParams') && isfield(sceneCfg.bucketParams, 'wallThickness')
    wallThickness = sceneCfg.bucketParams.wallThickness;
else
    wallThickness = 0.02;
end

figure('Name', 'App2-Prep: Perception + Obstacle Visualization', 'Color', 'w', 'Position', [80, 60, 1400, 760]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

ax1 = nexttile(1);
pcshow(ptCloud, 'Parent', ax1, 'MarkerSize', 35);
grid(ax1, 'on');
axis(ax1, 'equal');
view(ax1, 3);
title(ax1, 'Raw Bucket Point Cloud');
xlabel(ax1, 'X'); ylabel(ax1, 'Y'); zlabel(ax1, 'Z');

ax2 = nexttile(2);
pcshow(ptCloud, 'Parent', ax2, 'MarkerSize', 35);
hold(ax2, 'on');
drawBucketObstacle(ax2, features.topPoint, features.topRadius, features.bottomRadius, features.depth, wallThickness);

qScale = max(features.maxRadius, 0.2);
quiver3(ax2, features.topPoint(1), features.topPoint(2), features.topPoint(3), ...
    qScale*features.digDirection(1), qScale*features.digDirection(2), qScale*features.digDirection(3), ...
    0, 'LineWidth', 2.5, 'Color', [1 0 0]);
text(ax2, features.topPoint(1), features.topPoint(2), features.topPoint(3) + 0.06, 'digDirection', ...
    'Color', [0.95 0.2 0.2], 'FontWeight', 'bold');

title(ax2, 'Perceived Obstacle Model (Thin-Wall Bucket)');
xlabel(ax2, 'X'); ylabel(ax2, 'Y'); zlabel(ax2, 'Z');
grid(ax2, 'on'); axis(ax2, 'equal'); view(ax2, 3);

infoStr = sprintf(['top=[%.3f %.3f %.3f]\n' ...
    'rTop=%.3f rBottom=%.3f depth=%.3f maxR=%.3f\n' ...
    'dir=[%.3f %.3f %.3f]'], ...
    features.topPoint(1), features.topPoint(2), features.topPoint(3), ...
    features.topRadius, features.bottomRadius, features.depth, features.maxRadius, ...
    features.digDirection(1), features.digDirection(2), features.digDirection(3));
annotation('textbox', [0.67, 0.02, 0.31, 0.15], 'String', infoStr, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', [0.6 0.6 0.6]);

perception_result = struct();
perception_result.features = features;
perception_result.wallThickness = wallThickness;
perception_result.sourcePcd = pcdPath;
perception_result.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
save(outPath, '-struct', 'perception_result');

fprintf('[main.02] Saved perception result: %s\n', outPath);
end

function drawBucketObstacle(ax, topCenter, topRadius, bottomRadius, depth, wallThickness)
nTheta = 72;
nH = 28;
theta = linspace(0, 2*pi, nTheta);
h = linspace(0, depth, nH);
[TH, HH] = meshgrid(theta, h);

rNom = topRadius + (bottomRadius - topRadius) .* (HH / max(depth, 1e-9));
rOuter = rNom + wallThickness/2;
rInner = max(rNom - wallThickness/2, 0.001);

Xo = topCenter(1) + rOuter .* cos(TH);
Yo = topCenter(2) + rOuter .* sin(TH);
Zo = topCenter(3) - HH;
Xi = topCenter(1) + rInner .* cos(TH);
Yi = topCenter(2) + rInner .* sin(TH);
Zi = Zo;

surf(ax, Xo, Yo, Zo, 'FaceColor', [0.95 0.3 0.3], 'FaceAlpha', 0.16, 'EdgeAlpha', 0.05, 'EdgeColor', [0.9 0.2 0.2]);
surf(ax, Xi, Yi, Zi, 'FaceColor', [0.8 1.0 0.9], 'FaceAlpha', 0.10, 'EdgeAlpha', 0.04, 'EdgeColor', [0.2 0.8 0.3]);

% Bottom cap (outer)
rBottomOuter = bottomRadius + wallThickness/2;
t = linspace(0, 2*pi, 140);
xb = topCenter(1) + rBottomOuter * cos(t);
yb = topCenter(2) + rBottomOuter * sin(t);
zb = (topCenter(3) - depth) * ones(size(t));
plot3(ax, xb, yb, zb, '-', 'Color', [1 0.3 0.3], 'LineWidth', 1.8);
end
