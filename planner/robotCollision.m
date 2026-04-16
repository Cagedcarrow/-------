function [isColliding, info] = robotCollision(robot, configA, varargin)
%ROBOTCOLLISION Unified full self-collision checker.
% Usage:
%   [hit, info] = robotCollision(robot, q)
%   [hit, info] = robotCollision(robot, qA, qB)
%   [hit, info] = robotCollision(robot, qA, qB, maxStepRad)

    if nargin < 3
        [isColliding, info] = checkConfigCollision(robot, configA);
        return;
    end

    configB = varargin{1};
    maxStepRad = deg2rad(2.0);
    if numel(varargin) >= 2 && ~isempty(varargin{2})
        maxStepRad = varargin{2};
    end

    delta = configB - configA;
    n = max(2, ceil(max(abs(delta)) / max(maxStepRad, 1e-6)) + 1);

    isColliding = false;
    info = defaultInfo();
    info.mode = 'motion';

    for k = 1:n
        alpha = (k - 1) / max(n - 1, 1);
        cfg = configA + alpha * delta;
        [hitNow, cfgInfo] = checkConfigCollision(robot, cfg);

        if cfgInfo.min_distance < info.min_distance
            info.min_distance = cfgInfo.min_distance;
            info.body_a = cfgInfo.body_a;
            info.body_b = cfgInfo.body_b;
        end

        if hitNow
            isColliding = true;
            info = cfgInfo;
            info.mode = 'motion';
            info.sample_index = k;
            info.method = ['swept_' cfgInfo.method];
            return;
        end
    end
end

function [isColliding, info] = checkConfigCollision(robot, config)
    [isColliding, details] = checkCollision(robot, config, ...
        'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');

    info = defaultInfo();
    info.mode = 'config';
    info.min_distance = minFiniteDistance(details);

    if ~isColliding
        return;
    end

    labels = getCollisionBodyLabels(robot, details);
    [i, j] = firstCollisionPair(details);
    if ~isempty(i)
        info.body_a = char(labels(i));
        info.body_b = char(labels(j));
    end
    info.method = 'checkCollision';
    if ~isfinite(info.min_distance)
        info.min_distance = 0;
    end
end

function info = defaultInfo()
    info = struct();
    info.mode = 'config';
    info.method = 'none';
    info.body_a = '';
    info.body_b = '';
    info.min_distance = inf;
    info.sample_index = 0;
end

function bodyLabels = getCollisionBodyLabels(robot, details)
    baseLabel = "base";
    if isprop(robot, 'BaseName') && ~isempty(robot.BaseName)
        baseLabel = string(robot.BaseName);
    end

    bodyLabels = [baseLabel, string(robot.BodyNames)];
    if nargin >= 2 && size(details, 1) ~= numel(bodyLabels)
        bodyLabels = string(robot.BodyNames);
    end
end

function [rowIdx, colIdx] = firstCollisionPair(details)
    rowIdx = [];
    colIdx = [];
    n = size(details, 1);
    m = size(details, 2);
    if n == 0 || m == 0
        return;
    end

    maxJ = min(n, m);
    for i = 1:n
        for j = i+1:maxJ
            if isCollisionEntry(details(i, j)) || isCollisionEntry(details(j, i))
                rowIdx = i;
                colIdx = j;
                return;
            end
        end
    end
end

function dist = minFiniteDistance(details)
    dist = inf;
    if isempty(details) || ~isnumeric(details)
        return;
    end

    d = details;
    n = min(size(d, 1), size(d, 2));
    d(1:n+1:end) = inf;
    mask = isfinite(d) & ~isnan(d);
    if any(mask(:))
        dist = min(d(mask));
    end
end

function tf = isCollisionEntry(value)
    tf = false;
    if isempty(value)
        return;
    end
    if isnan(value)
        tf = true;
        return;
    end
    if isfinite(value) && value <= 0
        tf = true;
    end
end
