function [obstaclePoints, obstacleModels, collisionBoxes, data] = load_cloud_obstacles(matPath)
% load_cloud_obstacles:
% 读取 cloud_test 导出的 pcd_obstacles.mat，并生成 collisionBox 集合。
%
% 用法:
%   [obsPts, obsModels, obsBoxes, data] = load_cloud_obstacles();
%   [obsPts, obsModels, obsBoxes, data] = load_cloud_obstacles('xxx.mat');

if nargin < 1 || isempty(matPath)
    matPath = fullfile(fileparts(mfilename("fullpath")), "pcd_obstacles.mat");
end

fprintf("[load_cloud_obstacles] Load MAT: %s\n", matPath);
if ~isfile(matPath)
    error("[load_cloud_obstacles] 文件不存在: %s", matPath);
end

data = load(matPath);
mustFields = {"obstaclePoints", "obstacleModels"};
for i = 1:numel(mustFields)
    if ~isfield(data, mustFields{i})
        error("[load_cloud_obstacles] MAT缺少字段: %s", mustFields{i});
    end
end

obstaclePoints = data.obstaclePoints;
obstacleModels = data.obstacleModels;
collisionBoxes = cell(numel(obstacleModels), 1);

if exist('collisionBox', 'class') ~= 8
    warning("[load_cloud_obstacles] 未检测到collisionBox类，返回空collisionBoxes。");
    collisionBoxes = {};
    fprintf("[load_cloud_obstacles] obstaclePoints=%d, obstacleModels=%d\n", ...
        size(obstaclePoints, 1), numel(obstacleModels));
    return;
end

for i = 1:numel(obstacleModels)
    m = obstacleModels(i);
    box = collisionBox(m.size(1), m.size(2), m.size(3));
    box.Pose = trvec2tform(m.center);
    collisionBoxes{i} = box;
end

fprintf("[load_cloud_obstacles] obstaclePoints=%d, collisionBoxes=%d\n", ...
    size(obstaclePoints, 1), numel(collisionBoxes));
end
