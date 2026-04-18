tcp_frame_manual_check_main();

function tcp_frame_manual_check_main()
clc;
close all;

thisDir = fileparts(mfilename('fullpath'));
commonDir = fullfile(fileparts(thisDir), 'modules', 'common');
if exist('setup_project_paths', 'file') ~= 2
    addpath(commonDir);
end
ctx = setup_project_paths();

paths = struct();
paths.sceneCfg = fullfile(ctx.dataDir, 'scene_config.mat');
paths.perception = fullfile(ctx.dataDir, 'perception_result.mat');
paths.bucketPcd = fullfile(ctx.dataDir, 'bucket_scene.pcd');

if ~isfile(paths.sceneCfg)
    error('[main.04] Missing %s. Run main/app1_environment_setup.m first.', paths.sceneCfg);
end
if ~isfile(paths.perception)
    error('[main.04] Missing %s. Run main/app2_perceive_visualize_scene.m first.', paths.perception);
end

sceneCfg = load(paths.sceneCfg);
perception = load(paths.perception);
if isfield(sceneCfg, 'pcdFile') && isfile(sceneCfg.pcdFile)
    bucketPcdPath = sceneCfg.pcdFile;
else
    bucketPcdPath = paths.bucketPcd;
end
if ~isfile(bucketPcdPath)
    error('[main.04] Missing PCD file: %s', bucketPcdPath);
end
bucketPointCloud = pcread(bucketPcdPath);

loadOpts = struct();
loadOpts.urdfFile = ctx.urdfFile;
loadOpts.baseFrame = 'world_base';
loadOpts.baseLink = 'ur10';
loadOpts.endEffector = 'shovel_tip';
if isfield(sceneCfg, 'baseXYZ')
    loadOpts.baseXYZ = sceneCfg.baseXYZ;
else
    loadOpts.baseXYZ = [0, 0, 1.40];
end
if isfield(sceneCfg, 'baseYPR')
    loadOpts.baseYPR = sceneCfg.baseYPR;
else
    loadOpts.baseYPR = [0, 0, pi];
end

[robot, ~, jointLimits, qHome, robotInfo] = load_robot_and_base(loadOpts); %#ok<ASGLU>
ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [1 1 1 0.08 0.08 0.08];

features = perception.features;
wallThickness = 0.02;
if isfield(perception, 'wallThickness') && ~isempty(perception.wallThickness)
    wallThickness = perception.wallThickness;
elseif isfield(sceneCfg, 'bucketParams') && isfield(sceneCfg.bucketParams, 'wallThickness')
    wallThickness = sceneCfg.bucketParams.wallThickness;
end

bucketModel = struct();
bucketModel.topCenter = features.topPoint;
bucketModel.topRadius = features.topRadius;
bucketModel.bottomRadius = features.bottomRadius;
bucketModel.depth = features.depth;
bucketModel.wallThickness = wallThickness;
bucketModel.whitelistBodies = inferToolWhitelist(robot);

collisionOpts = struct('segmentSamples', 3, 'pointRadius', 0.018, 'verbose', false);
collisionFcn = @(q) collision_policy_bucket(robot, q, robotInfo.baseFrame, bucketModel, collisionOpts);

qCurrent = qHome;
refT = getTransform(robot, qCurrent, robotInfo.endEffector, robotInfo.baseFrame);
strictReject = true;
posRejectTol = 0.15;
oriRejectTolDeg = 30;
nJ = numel(qCurrent);
jointVals = qCurrent;
jointSliderSync = false;

vals = zeros(1, 6); % [dX dY dZ dYaw dPitch dRoll] in TCP frame
mins = [-0.8, -0.8, -0.8, -120, -120, -120];
maxs = [ 0.8,  0.8,  0.8,  120,  120,  120];
labels = {'dX_tcp (m)', 'dY_tcp (m)', 'dZ_tcp (m)', ...
    'dYaw_tcp (deg)', 'dPitch_tcp (deg)', 'dRoll_tcp (deg)'};

fig = figure('Name', 'App4 - TCP Frame Manual Check', 'Color', 'w', ...
    'Position', [70, 45, 1540, 860]);
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.04, 0.08, 0.68, 0.88]);
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
view(ax, 130, 25);
xlim(ax, [-2.2, 2.2]);
ylim(ax, [-2.2, 2.2]);
zlim(ax, [-0.3, 2.5]);

panelX = 0.75;
panelW = 0.22;
rowH = 0.064;
topY = 0.92;
sliders = gobjects(1, 6);
valueTexts = gobjects(1, 6);
jointSliders = gobjects(1, nJ);
jointValueTexts = gobjects(1, nJ);

for i = 1:6
    y = topY - (i - 1) * 0.075;
    uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [panelX, y, panelW, rowH * 0.45], ...
        'String', labels{i}, 'HorizontalAlignment', 'left', ...
        'BackgroundColor', get(fig, 'Color'));
    sliders(i) = uicontrol('Parent', fig, 'Style', 'slider', 'Units', 'normalized', ...
        'Position', [panelX, y - rowH * 0.45, panelW * 0.78, rowH * 0.45], ...
        'Min', mins(i), 'Max', maxs(i), 'Value', 0, ...
        'Callback', @(src, ~) onSliderChanged(i, src.Value));
    valueTexts(i) = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [panelX + panelW * 0.80, y - rowH * 0.45, panelW * 0.20, rowH * 0.45], ...
        'String', '0.000', 'HorizontalAlignment', 'left', ...
        'BackgroundColor', get(fig, 'Color'));
end

uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.33, 0.105, 0.050], 'String', 'Set TCP Ref', ...
    'FontWeight', 'bold', 'Callback', @onSetReference);
uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX + 0.115, 0.33, 0.105, 0.050], 'String', 'Reset Home', ...
    'Callback', @onResetHome);
uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.27, 0.220, 0.050], 'String', 'Zero Offsets', ...
    'Callback', @onZeroOffsets);

cbStrict = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [panelX, 0.235, panelW, 0.03], 'String', 'strictReject (collision/large IK err)', ...
    'Value', strictReject, 'BackgroundColor', get(fig, 'Color'), ...
    'Callback', @onToggleStrict);

txtStatus = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [panelX, 0.185, panelW, 0.045], 'String', 'Status: ready', ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(fig, 'Color'), ...
    'ForegroundColor', [0 0.45 0], 'FontSize', 10);

txtPose = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [panelX, 0.03, panelW, 0.145], 'String', '', ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(fig, 'Color'), ...
    'ForegroundColor', [0.20 0.20 0.20], 'FontSize', 9);

fprintf('[main.04] loaded scene: %s\n', bucketPcdPath);
fprintf('[main.04] strictReject=%d posTol=%.3f oriTol=%.1f\n', strictReject, posRejectTol, oriRejectTolDeg);
fprintf('[main.04] joint axes=%d (expect UR10=6)\n', nJ);
createJointControlWindow();
syncJointUIFromCurrent();
renderScene(refT, true, false);
updatePoseInfo(refT, 0.0, 0.0, false, 'init');

    function onSliderChanged(idx, value)
        vals(idx) = min(max(value, mins(idx)), maxs(idx));
        valueTexts(idx).String = sprintf('%.3f', vals(idx));
        applyTcpOffset();
    end

    function onSetReference(~, ~)
        refT = getTransform(robot, qCurrent, robotInfo.endEffector, robotInfo.baseFrame);
        resetOffsetsUI();
        syncJointUIFromCurrent();
        renderScene(refT, true, false);
        updatePoseInfo(refT, 0.0, 0.0, false, 'set_ref');
        setStatus('Status: TCP reference updated', [0 0.45 0]);
        p = tform2trvec(refT);
        fprintf('[main.04] set reference tcp=[%.3f %.3f %.3f]\n', p(1), p(2), p(3));
    end

    function onResetHome(~, ~)
        qCurrent = qHome;
        refT = getTransform(robot, qCurrent, robotInfo.endEffector, robotInfo.baseFrame);
        resetOffsetsUI();
        syncJointUIFromCurrent();
        renderScene(refT, true, false);
        updatePoseInfo(refT, 0.0, 0.0, false, 'reset_home');
        setStatus('Status: reset home + TCP reference', [0 0.45 0]);
        fprintf('[main.04] reset home done\n');
    end

    function onZeroOffsets(~, ~)
        resetOffsetsUI();
        applyTcpOffset();
    end

    function onToggleStrict(src, ~)
        strictReject = logical(src.Value);
        fprintf('[main.04] strictReject changed: %d\n', strictReject);
        applyTcpOffset();
    end

    function createJointControlWindow()
        figJ = figure('Name', 'App4 - Joint 6-Axis Control', 'Color', 'w', ...
            'Position', [1620, 80, 420, 760]); %#ok<NASGU>

        uicontrol('Parent', figJ, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.06, 0.955, 0.88, 0.035], ...
            'String', sprintf('Joint sliders (%d axes)', nJ), ...
            'HorizontalAlignment', 'left', 'BackgroundColor', get(figJ, 'Color'), ...
            'FontWeight', 'bold');

        yTop = 0.90;
        dY = 0.11;
        for j = 1:nJ
            y = yTop - (j - 1) * dY;
            lim = getJointSliderLimits(j);
            uicontrol('Parent', figJ, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.06, y, 0.88, 0.033], ...
                'String', sprintf('J%d (rad)', j), ...
                'HorizontalAlignment', 'left', 'BackgroundColor', get(figJ, 'Color'));
            jointSliders(j) = uicontrol('Parent', figJ, 'Style', 'slider', 'Units', 'normalized', ...
                'Position', [0.06, y - 0.040, 0.70, 0.038], ...
                'Min', lim(1), 'Max', lim(2), 'Value', qCurrent(j), ...
                'Callback', @(src, ~) onJointSliderChanged(j, src.Value));
            jointValueTexts(j) = uicontrol('Parent', figJ, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.78, y - 0.040, 0.16, 0.038], ...
                'String', '0.000', 'HorizontalAlignment', 'left', ...
                'BackgroundColor', get(figJ, 'Color'));
        end

        uicontrol('Parent', figJ, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.06, 0.05, 0.26, 0.055], 'String', 'Sync From Robot', ...
            'Callback', @(~, ~) syncJointUIFromCurrent());
        uicontrol('Parent', figJ, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.34, 0.05, 0.26, 0.055], 'String', 'Home', ...
            'Callback', @(~, ~) onResetHome([], []));
        uicontrol('Parent', figJ, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.62, 0.05, 0.32, 0.055], 'String', 'Set TCP Ref', ...
            'Callback', @(~, ~) onSetReference([], []));
    end

    function lim = getJointSliderLimits(j)
        lim = jointLimits(j, :);
        if ~all(isfinite(lim))
            lim = [-pi, pi];
        end
        if lim(2) <= lim(1) + 1e-6
            lim = [qCurrent(j) - pi, qCurrent(j) + pi];
        end
        if (lim(2) - lim(1)) > (4 * pi)
            lim = [max(lim(1), -2 * pi), min(lim(2), 2 * pi)];
        end
    end

    function onJointSliderChanged(idx, value)
        if jointSliderSync
            return;
        end
        jointVals(idx) = value;
        applyJointMove('joint_slider');
    end

    function applyJointMove(sourceTag)
        qTry = clamp_to_limits(jointVals, jointLimits);
        [colliding, colDetail] = collisionFcn(qTry);

        accept = true;
        reason = 'ok';
        if strictReject && colliding
            accept = false;
            reason = sprintf('collision (%s)', colDetail.firstHitBody);
        end

        if accept
            qCurrent = qTry;
            refT = getTransform(robot, qCurrent, robotInfo.endEffector, robotInfo.baseFrame);
            resetOffsetsUI();
            syncJointUIFromCurrent();
            renderScene(refT, true, colliding);
            updatePoseInfo(refT, 0.0, 0.0, colliding, sourceTag);
            if colliding
                setStatus('Status: joint accept (collision allowed by non-strict)', [0.88 0.45 0.05]);
            else
                setStatus('Status: joint accept', [0 0.45 0]);
            end
        else
            syncJointUIFromCurrent();
            renderScene(refT, false, colliding);
            setStatus(sprintf('Status: joint reject | %s', reason), [0.88 0.25 0.10]);
        end
        fprintf('[main.04] joint move source=%s accept=%d collide=%d qNorm=%.3f\n', ...
            sourceTag, accept, colliding, norm(qCurrent));
    end

    function syncJointUIFromCurrent()
        jointSliderSync = true;
        jointVals = qCurrent;
        for j = 1:nJ
            if isgraphics(jointSliders(j))
                jointSliders(j).Value = qCurrent(j);
            end
            if isgraphics(jointValueTexts(j))
                jointValueTexts(j).String = sprintf('%.3f', qCurrent(j));
            end
        end
        jointSliderSync = false;
    end

    function applyTcpOffset()
        dPosTCP = vals(1:3)';
        dEulTCP = deg2rad(vals(4:6));

        Rref = refT(1:3, 1:3);
        pref = refT(1:3, 4);
        Roff = eul2rotm(dEulTCP, 'ZYX');

        targetPos = pref + Rref * dPosTCP;
        targetRot = Rref * Roff;
        targetT = eye(4);
        targetT(1:3, 1:3) = targetRot;
        targetT(1:3, 4) = targetPos;

        [qTry, solInfo] = ik(robotInfo.endEffector, targetT, ikWeights, qCurrent);
        qTry = clamp_to_limits(qTry, jointLimits);
        qTry = nearest_equivalent_to_ref(qTry, qCurrent, jointLimits);

        Tnow = getTransform(robot, qTry, robotInfo.endEffector, robotInfo.baseFrame);
        posErr = norm(tform2trvec(Tnow) - targetPos');
        oriErr = rotm_geodesic_deg(tform2rotm(Tnow), tform2rotm(targetT));
        [colliding, colDetail] = collisionFcn(qTry);

        accept = true;
        reason = 'ok';
        if strictReject
            if colliding
                accept = false;
                reason = sprintf('collision (%s)', colDetail.firstHitBody);
            elseif posErr > posRejectTol
                accept = false;
                reason = sprintf('posErr %.3f > %.3f', posErr, posRejectTol);
            elseif oriErr > oriRejectTolDeg
                accept = false;
                reason = sprintf('oriErr %.2f > %.2f', oriErr, oriRejectTolDeg);
            end
        end

        if accept
            qCurrent = qTry;
            syncJointUIFromCurrent();
            renderScene(targetT, true, colliding);
            setStatus(sprintf('Status: accept | posErr=%.3f oriErr=%.2f', posErr, oriErr), [0 0.45 0]);
        else
            syncJointUIFromCurrent();
            renderScene(targetT, false, colliding);
            setStatus(sprintf('Status: reject | %s', reason), [0.88 0.25 0.10]);
        end

        updatePoseInfo(targetT, posErr, oriErr, colliding, char(solInfo.Status));
        fprintf(['[main.04] move d=[%.3f %.3f %.3f] eul=[%.1f %.1f %.1f] ' ...
            'accept=%d collide=%d posErr=%.4f oriErr=%.2f status=%s\n'], ...
            vals(1), vals(2), vals(3), vals(4), vals(5), vals(6), ...
            accept, colliding, posErr, oriErr, string(solInfo.Status));
    end

    function renderScene(targetT, acceptTarget, collidingTarget)
        cam = captureCameraState(ax);
        cla(ax);
        show(robot, qCurrent, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);
        restoreCameraState(ax, cam);
        hold(ax, 'on');
        pcshow(bucketPointCloud, 'Parent', ax, 'MarkerSize', 35);
        drawBucketThinWall(ax, bucketModel);

        Ttcp = getTransform(robot, qCurrent, robotInfo.endEffector, robotInfo.baseFrame);
        drawFrameAxes(ax, Ttcp, 0.13, 2.2, '-');

        if ~isempty(targetT)
            if acceptTarget
                markColor = [0.10 0.65 1.00];
            else
                markColor = [0.95 0.15 0.10];
            end
            if collidingTarget
                markColor = [0.92 0.10 0.10];
            end
            p = tform2trvec(targetT);
            plot3(ax, p(1), p(2), p(3), 'o', 'MarkerSize', 7, ...
                'MarkerFaceColor', markColor, 'MarkerEdgeColor', [0.1 0.1 0.1], 'LineWidth', 1.0);
            drawFrameAxes(ax, targetT, 0.10, 1.5, '--');
        end
        title(ax, 'App4: TCP-frame manual move check in perceived bucket scene');
        grid(ax, 'on');
        axis(ax, 'equal');
        drawnow limitrate;
    end

    function setStatus(textValue, colorValue)
        set(txtStatus, 'String', textValue, 'ForegroundColor', colorValue);
        drawnow limitrate;
    end

    function resetOffsetsUI()
        vals(:) = 0;
        for k = 1:6
            sliders(k).Value = 0;
            valueTexts(k).String = '0.000';
        end
    end

    function updatePoseInfo(Ttar, posErr, oriErr, colliding, ikStatus)
        p = tform2trvec(Ttar);
        txtPose.String = sprintf(['targetTCP:\n' ...
            '[%.3f %.3f %.3f]\n' ...
            'posErr=%.4f m\noriErr=%.2f deg\ncollide=%d\nIK=%s\n' ...
            'strictReject=%d'], ...
            p(1), p(2), p(3), posErr, oriErr, colliding, ikStatus, strictReject);
    end
end

function drawBucketThinWall(ax, b)
t = linspace(0, 2 * pi, 120);
xt = b.topCenter(1) + (b.topRadius + b.wallThickness / 2) * cos(t);
yt = b.topCenter(2) + (b.topRadius + b.wallThickness / 2) * sin(t);
zt = b.topCenter(3) * ones(size(t));
xb = b.topCenter(1) + (b.bottomRadius + b.wallThickness / 2) * cos(t);
yb = b.topCenter(2) + (b.bottomRadius + b.wallThickness / 2) * sin(t);
zb = (b.topCenter(3) - b.depth) * ones(size(t));
plot3(ax, xt, yt, zt, 'm-', 'LineWidth', 1.8);
plot3(ax, xb, yb, zb, 'm-', 'LineWidth', 1.8);
for k = 1:8
    id = round(1 + (k - 1) * (numel(t) - 1) / 8);
    plot3(ax, [xt(id), xb(id)], [yt(id), yb(id)], [zt(id), zb(id)], 'm-', 'LineWidth', 1.2);
end
end

function drawFrameAxes(ax, T, len, lw, style)
R = T(1:3, 1:3);
p = T(1:3, 4);
px = p + len * R(:, 1);
py = p + len * R(:, 2);
pz = p + len * R(:, 3);
plot3(ax, [p(1), px(1)], [p(2), px(2)], [p(3), px(3)], style, 'Color', [0.95 0.15 0.15], 'LineWidth', lw);
plot3(ax, [p(1), py(1)], [p(2), py(2)], [p(3), py(3)], style, 'Color', [0.10 0.80 0.20], 'LineWidth', lw);
plot3(ax, [p(1), pz(1)], [p(2), pz(2)], [p(3), pz(3)], style, 'Color', [0.15 0.45 1.00], 'LineWidth', lw);
end

function names = inferToolWhitelist(robot)
allNames = robot.BodyNames;
mask = false(size(allNames));
for i = 1:numel(allNames)
    n = allNames{i};
    mask(i) = contains(lower(n), 'shovel') || contains(n, '铲') || strcmp(n, 'ur10_wrist_3');
end
names = allNames(mask);
if isempty(names)
    names = {'shovel_tip'};
end
fprintf('[main.04] whitelist bodies: %s\n', strjoin(names, ', '));
end

function s = captureCameraState(ax)
s = struct();
s.CameraPosition = ax.CameraPosition;
s.CameraTarget = ax.CameraTarget;
s.CameraUpVector = ax.CameraUpVector;
s.CameraViewAngle = ax.CameraViewAngle;
end

function restoreCameraState(ax, s)
ax.CameraPositionMode = 'manual';
ax.CameraTargetMode = 'manual';
ax.CameraUpVectorMode = 'manual';
ax.CameraViewAngleMode = 'manual';
ax.CameraPosition = s.CameraPosition;
ax.CameraTarget = s.CameraTarget;
ax.CameraUpVector = s.CameraUpVector;
ax.CameraViewAngle = s.CameraViewAngle;
end
