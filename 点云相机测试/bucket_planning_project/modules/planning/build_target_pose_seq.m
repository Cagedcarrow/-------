function [T_target_seq, poseInfo] = build_target_pose_seq(pathBase, postureParams, opts)
% build_target_pose_seq:
% Build target pose sequence from a cartesian path.
% Tool y-axis follows path tangent, local Y compensation is applied.

if nargin < 3
    opts = struct();
end
if ~isfield(opts, 'flipToolZ')
    opts.flipToolZ = true;
end
if ~isfield(opts, 'xRef')
    opts.xRef = [0; 1; 0];
end

if ~isfield(postureParams, 'attackDeg')
    postureParams.attackDeg = 0;
end
if ~isfield(postureParams, 'assemblyDeg')
    postureParams.assemblyDeg = 0;
end
if ~isfield(postureParams, 'flipToolZ')
    postureParams.flipToolZ = opts.flipToolZ;
end

n = size(pathBase, 1);
T_target_seq = repmat(eye(4), 1, 1, n);
tanVec = zeros(n, 3);

thetaCompRad = deg2rad(postureParams.attackDeg + postureParams.assemblyDeg);
Rcomp = axis_angle_rotm([0; 1; 0], thetaCompRad);

prevX = [];
prevZ = [];
for i = 1:n
    if i == 1
        t = pathBase(min(i + 1, n), :) - pathBase(i, :);
    elseif i == n
        t = pathBase(i, :) - pathBase(max(i - 1, 1), :);
    else
        t = pathBase(i + 1, :) - pathBase(i - 1, :);
    end

    if norm(t) < 1e-10
        t = [1, 0, 0];
    end
    yAxis = (t(:) / norm(t));
    xRef = opts.xRef(:);
    zAxis = cross(xRef, yAxis);
    if norm(zAxis) < 1e-10
        zAxis = [0; 0; 1];
    end
    zAxis = zAxis / norm(zAxis);

    if postureParams.flipToolZ
        zAxis = -zAxis;
    end
    xAxis = cross(yAxis, zAxis);
    xAxis = xAxis / max(norm(xAxis), 1e-12);

    if ~isempty(prevX) && (dot(xAxis, prevX) < 0 || dot(zAxis, prevZ) < 0)
        xAxis = -xAxis;
        zAxis = -zAxis;
    end
    prevX = xAxis;
    prevZ = zAxis;

    Rnom = [xAxis, yAxis, zAxis];
    Rtar = Rnom * Rcomp;

    T = eye(4);
    T(1:3, 1:3) = Rtar;
    T(1:3, 4) = pathBase(i, :)';
    T_target_seq(:, :, i) = T;
    tanVec(i, :) = yAxis';
end

poseInfo = struct();
poseInfo.tangent = tanVec;
poseInfo.attackDeg = postureParams.attackDeg;
poseInfo.assemblyDeg = postureParams.assemblyDeg;
end

