build_bucket_scene_main();

function build_bucket_scene_main()
clc;
close all;

thisDir = fileparts(mfilename('fullpath'));
commonDir = fullfile(fileparts(thisDir), 'modules', 'common');
if exist('setup_project_paths', 'file') ~= 2
    addpath(commonDir);
end
ctx = setup_project_paths();

baseCfg = struct();
baseCfg.urdfFile = ctx.urdfFile;
baseCfg.baseFrame = 'world_base';
baseCfg.baseLink = 'ur10';
baseCfg.endEffector = 'shovel_tip';
baseCfg.baseXYZ = [0, 0, 1.40];
baseCfg.baseYPR = [0, 0, pi];

[robot, ~, jointLimits, qHome, robotInfo] = load_robot_and_base(baseCfg); %#ok<ASGLU>
[reachRadiusDefault, reachInfo] = estimate_reach_radius_from_urdf( ...
    robot, robotInfo.baseLink, robotInfo.endEffector, struct('sampleCount', 2600, 'seed', 11));

params = struct();
params.baseX = baseCfg.baseXYZ(1);
params.baseY = baseCfg.baseXYZ(2);
params.baseZ = baseCfg.baseXYZ(3);
params.topX = 1.00;
params.topY = 0.00;
params.topZ = 0.26;
params.topRadius = 0.34;
params.bottomRadius = 0.25;
params.depth = 0.48;
params.wallThickness = 0.02;

scenePaths = struct();
scenePaths.pcd = fullfile(ctx.dataDir, 'bucket_scene.pcd');
scenePaths.cfg = fullfile(ctx.dataDir, 'scene_config.mat');

fig = figure('Name', 'App1 - Build Bucket Scene', 'Color', 'w', 'Position', [60, 40, 1450, 840]);
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.04, 0.08, 0.68, 0.88]);
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
view(ax, 132, 24);
xlim(ax, [-2.2, 2.2]);
ylim(ax, [-2.2, 2.2]);
zlim(ax, [-0.3, 2.5]);

panelX = 0.75;
panelW = 0.22;
rowH = 0.064;

sBaseX = createSliderControl(fig, panelX, 0.92, panelW, rowH, 'baseX', -1.5, 1.5, params.baseX, @(v) onSet('baseX', v));
sBaseY = createSliderControl(fig, panelX, 0.85, panelW, rowH, 'baseY', -1.5, 1.5, params.baseY, @(v) onSet('baseY', v));
sBaseZ = createSliderControl(fig, panelX, 0.78, panelW, rowH, 'baseZ', 0.40, 2.10, params.baseZ, @(v) onSet('baseZ', v));
sTopX = createSliderControl(fig, panelX, 0.70, panelW, rowH, 'topX', -1.6, 1.6, params.topX, @(v) onSet('topX', v));
sTopY = createSliderControl(fig, panelX, 0.63, panelW, rowH, 'topY', -1.6, 1.6, params.topY, @(v) onSet('topY', v));
sTopZ = createSliderControl(fig, panelX, 0.56, panelW, rowH, 'topZ', -0.2, 1.6, params.topZ, @(v) onSet('topZ', v));
sTopR = createSliderControl(fig, panelX, 0.48, panelW, rowH, 'topRadius', 0.10, 0.85, params.topRadius, @(v) onSet('topRadius', v));
sBotR = createSliderControl(fig, panelX, 0.41, panelW, rowH, 'bottomRadius', 0.08, 0.80, params.bottomRadius, @(v) onSet('bottomRadius', v));
sDepth = createSliderControl(fig, panelX, 0.34, panelW, rowH, 'depth', 0.10, 1.30, params.depth, @(v) onSet('depth', v));
sWall = createSliderControl(fig, panelX, 0.27, panelW, rowH, 'wallThickness', 0.005, 0.08, params.wallThickness, @(v) onSet('wallThickness', v));

btnSave = uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.17, panelW, 0.06], 'String', 'Generate && Save', ...
    'FontSize', 12, 'FontWeight', 'bold', 'Callback', @onSave);
txtStatus = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [panelX, 0.10, panelW, 0.06], 'String', 'Status: Ready', ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(fig, 'Color'), 'ForegroundColor', [0 0.45 0], ...
    'FontSize', 10);

txtInfo = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [panelX, 0.03, panelW, 0.07], ...
    'String', sprintf('reachRadius=%.3f m (samples=%d)', reachRadiusDefault, reachInfo.sampleCount), ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(fig, 'Color'), 'ForegroundColor', [0.2 0.2 0.2], ...
    'FontSize', 9);

refreshPreview();

    function onSet(name, value)
        params.(name) = value;
        refreshPreview();
    end

    function refreshPreview()
        if params.bottomRadius >= params.topRadius
            set(txtStatus, 'String', 'Status: warning, bottomRadius >= topRadius', 'ForegroundColor', [0.85 0.4 0]);
        else
            set(txtStatus, 'String', 'Status: preview updated', 'ForegroundColor', [0 0.45 0]);
        end

        % Update robot base.
        baseXYZ = [params.baseX, params.baseY, params.baseZ];
        Tbase = trvec2tform(baseXYZ) * eul2tform(baseCfg.baseYPR, 'ZYX');
        baseBody = getBody(robot, robotInfo.baseLink);
        setFixedTransform(baseBody.Joint, Tbase);

        bucketParams = getBucketParamsFromUI();
        bucketPreview = generate_bucket_pcd(bucketParams, '', struct( ...
            'nSide', 2200, 'nBottom', 700, 'nRim', 600, 'noiseSigma', 0.0, 'seed', 3));

        cla(ax);
        show(robot, qHome, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);
        hold(ax, 'on');
        pcshow(bucketPreview.ptCloud, 'Parent', ax, 'MarkerSize', 35);

        TbaseNow = getTransform(robot, qHome, robotInfo.baseLink, robotInfo.baseFrame);
        c = tform2trvec(TbaseNow);
        [sx, sy, sz] = sphere(30);
        surf(ax, c(1) + reachRadiusDefault*sx, c(2) + reachRadiusDefault*sy, c(3) + reachRadiusDefault*sz, ...
            'FaceColor', [0.2, 0.7, 1.0], 'FaceAlpha', 0.10, 'EdgeColor', [0.2, 0.6, 0.9], 'EdgeAlpha', 0.18);
        plot3(ax, c(1), c(2), c(3), 'ms', 'MarkerSize', 7, 'LineWidth', 1.6);

        dToBucket = norm(bucketParams.topCenter - c) + max(bucketParams.topRadius, bucketParams.bottomRadius);
        inReach = dToBucket <= reachRadiusDefault;
        if inReach
            set(txtStatus, 'String', sprintf('Status: reachable (%.3f <= %.3f)', dToBucket, reachRadiusDefault), ...
                'ForegroundColor', [0 0.45 0]);
        else
            set(txtStatus, 'String', sprintf('Status: out of reach (%.3f > %.3f)', dToBucket, reachRadiusDefault), ...
                'ForegroundColor', [0.90 0.15 0.10]);
        end

        title(ax, 'App1: environment setup + bucket PCD generation');
        drawnow;
    end

    function onSave(~, ~)
        bucketParams = getBucketParamsFromUI();
        saveStats = generate_bucket_pcd(bucketParams, scenePaths.pcd, struct( ...
            'nSide', 18000, 'nBottom', 6000, 'nRim', 3200, 'noiseSigma', 0.0012, 'seed', 7));

        baseXYZ = [params.baseX, params.baseY, params.baseZ];
        T_world_to_baseLink = trvec2tform(baseXYZ) * eul2tform(baseCfg.baseYPR, 'ZYX');

        scene_config = struct();
        scene_config.T_world_to_baseLink = T_world_to_baseLink;
        scene_config.baseXYZ = baseXYZ;
        scene_config.baseYPR = baseCfg.baseYPR;
        scene_config.bucketParams = bucketParams;
        scene_config.reachRadiusDefault = reachRadiusDefault;
        scene_config.reachInfo = reachInfo;
        scene_config.pcdFile = scenePaths.pcd;
        scene_config.frame = baseCfg.baseFrame;
        scene_config.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
        scene_config.pointCount = saveStats.pointCount;

        save(scenePaths.cfg, '-struct', 'scene_config');
        fprintf('[main.01] Saved %s and %s\n', scenePaths.pcd, scenePaths.cfg);

        set(txtStatus, 'String', sprintf('Status: saved (%d points)', saveStats.pointCount), 'ForegroundColor', [0 0.45 0]);
    end

    function p = getBucketParamsFromUI()
        p = struct();
        p.topCenter = [params.topX, params.topY, params.topZ];
        p.topRadius = params.topRadius;
        p.bottomRadius = params.bottomRadius;
        p.depth = params.depth;
        p.wallThickness = params.wallThickness;
    end
end

function out = createSliderControl(parentFig, px, py, pw, rh, label, vmin, vmax, v0, cb)
uicontrol('Parent', parentFig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [px, py, pw, rh*0.45], 'String', label, 'HorizontalAlignment', 'left', ...
    'BackgroundColor', get(parentFig, 'Color'));

out.slider = uicontrol('Parent', parentFig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [px, py-rh*0.45, pw*0.78, rh*0.45], ...
    'Min', vmin, 'Max', vmax, 'Value', v0, ...
    'Callback', @(src,~) onChange(src));

out.valueText = uicontrol('Parent', parentFig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [px+pw*0.80, py-rh*0.45, pw*0.20, rh*0.45], ...
    'String', sprintf('%.3f', v0), 'HorizontalAlignment', 'left', ...
    'BackgroundColor', get(parentFig, 'Color'));

    function onChange(src)
        val = src.Value;
        out.valueText.String = sprintf('%.3f', val);
        cb(val);
    end
end
