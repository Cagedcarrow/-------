function [isColliding, detail] = collision_policy_bucket(robot, q, baseFrame, bucketModel, opts)
% collision_policy_bucket:
% Collision policy for thin-wall bucket.
%
% Rules:
% 1) Bucket wall is forbidden for all links.
% 2) Bucket inner cavity is allowed only for whitelist tool bodies.

if nargin < 5
    opts = struct();
end
if ~isfield(opts, 'segmentSamples')
    opts.segmentSamples = 2;
end
if ~isfield(opts, 'pointRadius')
    opts.pointRadius = 0.015;
end
if ~isfield(opts, 'verbose')
    opts.verbose = false;
end

if ~isfield(bucketModel, 'whitelistBodies') || isempty(bucketModel.whitelistBodies)
    bucketModel.whitelistBodies = {'shovel_tip'};
end

[samplePts, sampleBodyNames] = sampleRobotPoints(robot, q, baseFrame, opts.segmentSamples);

collideCount = 0;
interiorCount = 0;
wallCount = 0;
bottomCount = 0;
firstHit = '';

for i = 1:size(samplePts, 1)
    p = samplePts(i, :);
    bodyName = sampleBodyNames{i};
    allowedInside = any(strcmp(bodyName, bucketModel.whitelistBodies));
    [isHit, hitType] = checkPointAgainstBucket(p, bucketModel, allowedInside, opts.pointRadius);
    if isHit
        collideCount = collideCount + 1;
        if isempty(firstHit)
            firstHit = bodyName;
        end
        switch hitType
            case 'interior'
                interiorCount = interiorCount + 1;
            case 'wall'
                wallCount = wallCount + 1;
            case 'bottom'
                bottomCount = bottomCount + 1;
        end
    end
end

isColliding = collideCount > 0;
detail = struct();
detail.collideCount = collideCount;
detail.interiorCount = interiorCount;
detail.wallCount = wallCount;
detail.bottomCount = bottomCount;
detail.firstHitBody = firstHit;

if opts.verbose && isColliding
    fprintf('[control.collision_policy] hit=%d wall=%d interior=%d bottom=%d first=%s\n', ...
        collideCount, wallCount, interiorCount, bottomCount, firstHit);
end
end

function [pts, bodyNames] = sampleRobotPoints(robot, q, baseFrame, segmentSamples)
bodyList = robot.BodyNames;
nBodies = numel(bodyList);

pts = zeros(0, 3);
bodyNames = cell(0, 1);

for i = 1:nBodies
    childName = bodyList{i};
    Tchild = getTransform(robot, q, childName, baseFrame);
    pChild = tform2trvec(Tchild);

    pts = [pts; pChild]; %#ok<AGROW>
    bodyNames{end + 1, 1} = childName; %#ok<AGROW>

    bodyObj = getBody(robot, childName);
    parentName = bodyObj.Parent;
    if isempty(parentName) || strcmp(parentName, robot.BaseName)
        continue;
    end

    Tparent = getTransform(robot, q, parentName, baseFrame);
    pParent = tform2trvec(Tparent);
    for k = 1:segmentSamples
        alpha = k / (segmentSamples + 1);
        p = (1 - alpha) * pParent + alpha * pChild;
        pts = [pts; p]; %#ok<AGROW>
        bodyNames{end + 1, 1} = childName; %#ok<AGROW>
    end
end
end

function [isHit, hitType] = checkPointAgainstBucket(p, b, allowInside, pointRadius)
% Bucket local convention:
% topCenter at h=0, positive h downward.
rel = p - b.topCenter;
h = -rel(3);
rho = hypot(rel(1), rel(2));

isHit = false;
hitType = '';

% Quick reject around vertical bounds.
if h < -pointRadius || h > (b.depth + b.wallThickness + pointRadius)
    return;
end

if h >= 0 && h <= b.depth
    frac = h / max(b.depth, 1e-9);
    rNom = b.topRadius + (b.bottomRadius - b.topRadius) * frac;
    rOuter = rNom + b.wallThickness / 2 + pointRadius;
    rInner = max(rNom - b.wallThickness / 2 - pointRadius, 0);

    inWall = (rho >= rInner) && (rho <= rOuter);
    inInterior = rho < rInner;

    if inWall
        isHit = true;
        hitType = 'wall';
        return;
    end

    if (~allowInside) && inInterior
        isHit = true;
        hitType = 'interior';
        return;
    end
end

% Bottom cap region.
if h > b.depth && h <= (b.depth + b.wallThickness + pointRadius)
    rBottomOuter = b.bottomRadius + b.wallThickness / 2 + pointRadius;
    if rho <= rBottomOuter
        isHit = true;
        hitType = 'bottom';
    end
end
end
