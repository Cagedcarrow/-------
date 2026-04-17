clc;
clear;
close all;

% cloud_test:
% 读取点云 -> 分割地面 -> 聚类障碍 -> 构建包围盒 -> 导出规划数据
% 可视化: 黑色背景, 单窗口全屏展示 [原始点云 | 障碍构建结果]

cfg = struct();

cfg.source = struct();
cfg.source.type = "file"; % "file" | "camera"
cfg.source.path = fullfile(fileparts(mfilename("fullpath")), "synthetic_scene.pcd");

cfg.proc = struct();
cfg.proc.gridStep = 0.03;
cfg.proc.roiX = [-2.0, 2.0];
cfg.proc.roiY = [-2.0, 2.0];
cfg.proc.roiZ = [-0.1, 2.0];
cfg.proc.maxGroundDist = 0.03;
cfg.proc.minClusterPts = 40;
cfg.proc.clusterDist = 0.12;

cfg.build = struct();
cfg.build.inflateMargin = 0.03;
cfg.build.minBoxSize = [0.08, 0.08, 0.08];
cfg.build.surfaceSampleStep = 0.04;
cfg.build.plannerPointSource = "box"; % "raw" | "box"

cfg.export = struct();
cfg.export.enable = true;
cfg.export.matPath = fullfile(fileparts(mfilename("fullpath")), "pcd_obstacles.mat");
cfg.export.pcdPath = fullfile(fileparts(mfilename("fullpath")), "obstacle_points_for_planner.pcd");

fprintf("[cloud_test] Start point cloud loading...\n");
ptCloudRaw = loadPointCloud(cfg.source);
fprintf("[cloud_test] Raw points: %d\n", ptCloudRaw.Count);

ptCloudDS = pcdownsample(ptCloudRaw, "gridAverage", cfg.proc.gridStep);
fprintf("[cloud_test] After downsample: %d\n", ptCloudDS.Count);

xyz = ptCloudDS.Location;
roiMask = xyz(:, 1) >= cfg.proc.roiX(1) & xyz(:, 1) <= cfg.proc.roiX(2) & ...
          xyz(:, 2) >= cfg.proc.roiY(1) & xyz(:, 2) <= cfg.proc.roiY(2) & ...
          xyz(:, 3) >= cfg.proc.roiZ(1) & xyz(:, 3) <= cfg.proc.roiZ(2);
ptCloudROI = select(ptCloudDS, find(roiMask));
fprintf("[cloud_test] After ROI crop: %d\n", ptCloudROI.Count);

if ptCloudROI.Count < 50
    error("[cloud_test] ROI内点数过少，请检查点云文件或ROI参数。");
end

[groundModel, inlierIdx, ~] = pcfitplane(ptCloudROI, cfg.proc.maxGroundDist, [0, 0, 1], 10);
groundCloud = select(ptCloudROI, inlierIdx);
obstacleCloud = select(ptCloudROI, setdiff(1:ptCloudROI.Count, inlierIdx));

fprintf("[cloud_test] Ground points: %d\n", groundCloud.Count);
fprintf("[cloud_test] Obstacle candidate points: %d\n", obstacleCloud.Count);
fprintf("[cloud_test] Ground model: %.4fx + %.4fy + %.4fz + %.4f = 0\n", ...
    groundModel.Parameters(1), groundModel.Parameters(2), groundModel.Parameters(3), groundModel.Parameters(4));

if obstacleCloud.Count == 0
    error("[cloud_test] obstacleCloud为空，无法构建障碍物。");
end

[labels, numClusters] = pcsegdist(obstacleCloud, cfg.proc.clusterDist);
clusterCounts = zeros(numClusters, 1);
for i = 1:numClusters
    clusterCounts(i) = sum(labels == i);
end
validClusters = find(clusterCounts >= cfg.proc.minClusterPts);
fprintf("[cloud_test] Found clusters: %d, valid clusters (>= %d pts): %d\n", ...
    numClusters, cfg.proc.minClusterPts, numel(validClusters));

if isempty(validClusters)
    error("[cloud_test] 没有满足最小点数阈值的簇，无法构建障碍物。");
end

[obstacleModels, obstaclePointsRaw, obstaclePointsBox] = buildObstacleModels( ...
    obstacleCloud, labels, validClusters, cfg.build);

switch cfg.build.plannerPointSource
    case "raw"
        obstaclePoints = obstaclePointsRaw;
    case "box"
        obstaclePoints = obstaclePointsBox;
    otherwise
        error("[cloud_test] 未知plannerPointSource: %s", cfg.build.plannerPointSource);
end

fprintf("[cloud_test] Planner points source: %s, count=%d\n", ...
    cfg.build.plannerPointSource, size(obstaclePoints, 1));
for i = 1:numel(obstacleModels)
    m = obstacleModels(i);
    fprintf("[cloud_test] Obstacle #%d | pts=%d | center=[%.3f %.3f %.3f] | size=[%.3f %.3f %.3f]\n", ...
        i, m.numPoints, m.center(1), m.center(2), m.center(3), m.size(1), m.size(2), m.size(3));
end

if cfg.export.enable
    exportData = struct();
    exportData.sourcePath = char(cfg.source.path);
    exportData.generatedAt = char(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
    exportData.groundPlane = groundModel.Parameters;
    exportData.obstacleModels = obstacleModels;
    exportData.obstaclePoints = obstaclePoints;
    exportData.obstaclePointsRaw = obstaclePointsRaw;
    exportData.obstaclePointsBox = obstaclePointsBox;
    exportData.procCfg = cfg.proc;
    exportData.buildCfg = cfg.build;

    save(cfg.export.matPath, "-struct", "exportData");
    fprintf("[cloud_test] Saved MAT: %s\n", cfg.export.matPath);

    try
        pcwrite(pointCloud(obstaclePoints), cfg.export.pcdPath, "Encoding", "ascii");
        fprintf("[cloud_test] Saved planner PCD: %s\n", cfg.export.pcdPath);
    catch ME
        warning("[cloud_test] 写planner PCD失败: %s", ME.message);
    end
end

% 黑色背景 + 全屏 + 单窗口双图
fig = figure("Name", "Cloud Processing Result", "Color", "k", "WindowState", "maximized");
tiledlayout(fig, 1, 2, "Padding", "compact", "TileSpacing", "compact");

% 左: 原始三维点云
ax1 = nexttile;
pcshow(ptCloudRaw, "Parent", ax1);
setDarkAxesStyle(ax1);
title(ax1, "Raw 3D Point Cloud", "Color", "w");
xlabel(ax1, "X (m)"); ylabel(ax1, "Y (m)"); zlabel(ax1, "Z (m)");
view(ax1, 3); axis(ax1, "equal");

% 右: 障碍构建结果
ax2 = nexttile;
pcshow(groundCloud, "Parent", ax2, "MarkerSize", 40);
hold(ax2, "on");
obsXYZ = obstacleCloud.Location;
obsColor = uint8(repmat([255, 70, 70], size(obsXYZ, 1), 1));
pcshow(obsXYZ, obsColor, "Parent", ax2, "MarkerSize", 40);

for i = 1:numel(obstacleModels)
    drawAABB(ax2, obstacleModels(i).minBound, obstacleModels(i).maxBound, [0.2, 1.0, 0.2], 1.6);
    text(ax2, obstacleModels(i).center(1), obstacleModels(i).center(2), obstacleModels(i).maxBound(3) + 0.03, ...
        sprintf("#%d", i), "Color", [0.8, 1.0, 0.8], "FontWeight", "bold", "FontSize", 11);
end

setDarkAxesStyle(ax2);
title(ax2, "Obstacle Construction Result", "Color", "w");
xlabel(ax2, "X (m)"); ylabel(ax2, "Y (m)"); zlabel(ax2, "Z (m)");
view(ax2, 3); axis(ax2, "equal");

fprintf("[cloud_test] Done.\n");

function ptCloud = loadPointCloud(sourceCfg)
switch sourceCfg.type
    case "file"
        if ~isfile(sourceCfg.path)
            error("[loadPointCloud] 文件不存在: %s", sourceCfg.path);
        end
        [~, ~, ext] = fileparts(sourceCfg.path);
        ext = lower(char(ext));
        validExt = {'.pcd', '.ply', '.las', '.laz'};
        fprintf("[loadPointCloud] File ext = %s\n", ext);
        if ~any(strcmpi(ext, validExt))
            warning("[loadPointCloud] 扩展名%s未验证，尝试按点云文件读取。", ext);
        end
        ptCloud = pcread(sourceCfg.path);
    case "camera"
        error("[loadPointCloud] 目前未接入实时相机，请先使用file模式。");
    otherwise
        error("[loadPointCloud] 未知source.type: %s", sourceCfg.type);
end
end

function [obstacleModels, obstaclePointsRaw, obstaclePointsBox] = buildObstacleModels( ...
    obstacleCloud, labels, validClusters, buildCfg)
obsXYZ = obstacleCloud.Location;
n = numel(validClusters);

obstacleModels = repmat(struct( ...
    "id", 0, ...
    "clusterId", 0, ...
    "numPoints", 0, ...
    "minBound", zeros(1, 3), ...
    "maxBound", zeros(1, 3), ...
    "center", zeros(1, 3), ...
    "size", zeros(1, 3)), n, 1);

rawCell = cell(n, 1);
boxCell = cell(n, 1);

for i = 1:n
    cid = validClusters(i);
    idx = (labels == cid);
    pts = obsXYZ(idx, :);
    rawCell{i} = pts;

    minBound = min(pts, [], 1) - buildCfg.inflateMargin;
    maxBound = max(pts, [], 1) + buildCfg.inflateMargin;
    sizeVec = maxBound - minBound;
    sizeVec = max(sizeVec, buildCfg.minBoxSize);
    center = (minBound + maxBound) / 2;
    minBound = center - sizeVec / 2;
    maxBound = center + sizeVec / 2;

    boxCell{i} = sampleAABBPoints(minBound, maxBound, buildCfg.surfaceSampleStep);

    obstacleModels(i).id = i;
    obstacleModels(i).clusterId = cid;
    obstacleModels(i).numPoints = size(pts, 1);
    obstacleModels(i).minBound = minBound;
    obstacleModels(i).maxBound = maxBound;
    obstacleModels(i).center = center;
    obstacleModels(i).size = sizeVec;
end

obstaclePointsRaw = vertcat(rawCell{:});
obstaclePointsBox = unique(vertcat(boxCell{:}), "rows");
end

function pts = sampleAABBPoints(minBound, maxBound, step)
x = axisSamples(minBound(1), maxBound(1), step);
y = axisSamples(minBound(2), maxBound(2), step);
z = axisSamples(minBound(3), maxBound(3), step);

[Xxy, Yxy] = meshgrid(x, y);
faceZMin = [Xxy(:), Yxy(:), minBound(3) * ones(numel(Xxy), 1)];
faceZMax = [Xxy(:), Yxy(:), maxBound(3) * ones(numel(Xxy), 1)];

[Xxz, Zxz] = meshgrid(x, z);
faceYMin = [Xxz(:), minBound(2) * ones(numel(Xxz), 1), Zxz(:)];
faceYMax = [Xxz(:), maxBound(2) * ones(numel(Xxz), 1), Zxz(:)];

[Yyz, Zyz] = meshgrid(y, z);
faceXMin = [minBound(1) * ones(numel(Yyz), 1), Yyz(:), Zyz(:)];
faceXMax = [maxBound(1) * ones(numel(Yyz), 1), Yyz(:), Zyz(:)];

pts = unique([faceZMin; faceZMax; faceYMin; faceYMax; faceXMin; faceXMax], "rows");
end

function v = axisSamples(lo, hi, step)
if hi <= lo
    v = lo;
    return;
end
n = max(2, ceil((hi - lo) / step) + 1);
v = linspace(lo, hi, n)';
end

function drawAABB(ax, minBound, maxBound, color, lineWidth)
c = [ ...
    minBound(1), minBound(2), minBound(3); ...
    maxBound(1), minBound(2), minBound(3); ...
    maxBound(1), maxBound(2), minBound(3); ...
    minBound(1), maxBound(2), minBound(3); ...
    minBound(1), minBound(2), maxBound(3); ...
    maxBound(1), minBound(2), maxBound(3); ...
    maxBound(1), maxBound(2), maxBound(3); ...
    minBound(1), maxBound(2), maxBound(3)];

edges = [ ...
    1 2; 2 3; 3 4; 4 1; ...
    5 6; 6 7; 7 8; 8 5; ...
    1 5; 2 6; 3 7; 4 8];

for i = 1:size(edges, 1)
    id1 = edges(i, 1);
    id2 = edges(i, 2);
    plot3(ax, [c(id1, 1), c(id2, 1)], [c(id1, 2), c(id2, 2)], [c(id1, 3), c(id2, 3)], ...
        "-", "Color", color, "LineWidth", lineWidth);
end
end

function setDarkAxesStyle(ax)
set(ax, ...
    "Color", "k", ...
    "XColor", [1 1 1], ...
    "YColor", [1 1 1], ...
    "ZColor", [1 1 1], ...
    "GridColor", [0.4 0.4 0.4], ...
    "MinorGridColor", [0.25 0.25 0.25], ...
    "GridAlpha", 0.6, ...
    "MinorGridAlpha", 0.35);
grid(ax, "on");
end
