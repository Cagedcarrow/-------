plan_execute_rrtstar_main();

function plan_execute_rrtstar_main()
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

state = struct();
state.loaded = false;
state.sceneCfg = struct();
state.perception = struct();
state.robot = [];
state.jointLimits = [];
state.qCurrent = [];
state.robotInfo = struct();
state.bucketPointCloud = [];
state.bucketModel = struct();
state.pathBase = [];
state.T_target_seq = [];
state.keyPts = struct();

trajParams = struct();
trajParams.entrySpanFactor = 0.95;
trajParams.midSpanFactor = 0.35;
trajParams.exitSpanFactor = 0.90;
trajParams.entryLift = 0.04;
trajParams.entryDepthRatio = 0.45;
trajParams.deepDepthRatio = 0.88;
trajParams.exitDepthRatio = 0.22;
trajParams.midSlopeAtKnot1 = -0.20;
trajParams.localYOffset = 0.0;

postureParams = struct('attackDeg', -12.0, 'assemblyDeg', 0.0, 'flipToolZ', true);
nPts = 90;

fig = figure('Name', 'App3 - Plan && Execute (RRT* + IK)', 'Color', 'w', ...
    'Position', [70, 45, 1500, 860]);
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.04, 0.08, 0.68, 0.88]);
grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
view(ax, 130, 25);
xlim(ax, [-2.2, 2.2]); ylim(ax, [-2.2, 2.2]); zlim(ax, [-0.3, 2.5]);

panelX = 0.75;
panelW = 0.22;
rowH = 0.064;

btnLoad = uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.92, panelW, 0.05], 'String', 'Load Scene', ...
    'FontWeight', 'bold', 'Callback', @onLoadScene);
btnPreview = uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.86, panelW, 0.05], 'String', 'Preview Plan', ...
    'Callback', @onPreview);
btnRun = uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.80, panelW, 0.05], 'String', 'Plan && Execute', ...
    'FontSize', 12, 'FontWeight', 'bold', 'Callback', @onRun);

sAttack = createSliderControl(fig, panelX, 0.72, panelW, rowH, 'attackDeg', -45, 45, postureParams.attackDeg, ...
    @(v) onSetPosture('attackDeg', v));
sAssembly = createSliderControl(fig, panelX, 0.65, panelW, rowH, 'assemblyDeg', -180, 180, postureParams.assemblyDeg, ...
    @(v) onSetPosture('assemblyDeg', v));
sEntryD = createSliderControl(fig, panelX, 0.57, panelW, rowH, 'entryDepthRatio', 0.20, 0.90, trajParams.entryDepthRatio, ...
    @(v) onSetTraj('entryDepthRatio', v));
sDeepD = createSliderControl(fig, panelX, 0.50, panelW, rowH, 'deepDepthRatio', 0.40, 0.98, trajParams.deepDepthRatio, ...
    @(v) onSetTraj('deepDepthRatio', v));
sExitD = createSliderControl(fig, panelX, 0.43, panelW, rowH, 'exitDepthRatio', 0.05, 0.70, trajParams.exitDepthRatio, ...
    @(v) onSetTraj('exitDepthRatio', v));
sMidS = createSliderControl(fig, panelX, 0.36, panelW, rowH, 'midSlopeAtKnot1', -1.2, 0.3, trajParams.midSlopeAtKnot1, ...
    @(v) onSetTraj('midSlopeAtKnot1', v));

cbFlip = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [panelX, 0.29, panelW, 0.03], 'String', 'flipToolZ', ...
    'Value', postureParams.flipToolZ, 'BackgroundColor', get(fig, 'Color'), ...
    'Callback', @onFlip);

txtStatus = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [panelX, 0.22, panelW, 0.05], 'String', 'Status: waiting load', ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(fig, 'Color'), ...
    'ForegroundColor', [0 0.45 0], 'FontSize', 10);

txtLog = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [panelX, 0.05, panelW, 0.16], ...
    'String', sprintf('Input:\n%s\n%s', paths.sceneCfg, paths.perception), ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(fig, 'Color'), 'ForegroundColor', [0.25 0.25 0.25], ...
    'FontSize', 9);

    function onSetPosture(name, value)
        postureParams.(name) = value;
        if state.loaded
            onPreview();
        end
    end

    function onSetTraj(name, value)
        trajParams.(name) = value;
        if state.loaded
            onPreview();
        end
    end

    function onFlip(src, ~)
        postureParams.flipToolZ = logical(src.Value);
        if state.loaded
            onPreview();
        end
    end

    function onLoadScene(~, ~)
        set(txtStatus, 'String', 'Status: loading scene...', 'ForegroundColor', [0 0.3 0.8]);
        drawnow;

        if ~isfile(paths.sceneCfg)
            error('[main.03] Missing %s. Run main/app1_environment_setup.m first.', paths.sceneCfg);
        end
        if ~isfile(paths.perception)
            error('[main.03] Missing %s. Run main/app2_perceive_visualize_scene.m first.', paths.perception);
        end

        sceneCfg = load(paths.sceneCfg);
        perception = load(paths.perception);
        if isfield(sceneCfg, 'pcdFile') && isfile(sceneCfg.pcdFile)
            bucketPcdPath = sceneCfg.pcdFile;
        else
            bucketPcdPath = paths.bucketPcd;
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

        bucketFeatures = perception.features;
        wallThickness = perception.wallThickness;
        if isempty(wallThickness) || wallThickness <= 0
            wallThickness = 0.02;
        end

        bucketModel = struct();
        bucketModel.topCenter = bucketFeatures.topPoint;
        bucketModel.topRadius = bucketFeatures.topRadius;
        bucketModel.bottomRadius = bucketFeatures.bottomRadius;
        bucketModel.depth = bucketFeatures.depth;
        bucketModel.wallThickness = wallThickness;
        bucketModel.whitelistBodies = inferToolWhitelist(robot);

        state.loaded = true;
        state.sceneCfg = sceneCfg;
        state.perception = perception;
        state.robot = robot;
        state.jointLimits = jointLimits;
        state.qCurrent = qHome;
        state.robotInfo = robotInfo;
        state.bucketPointCloud = bucketPointCloud;
        state.bucketModel = bucketModel;

        renderBaseScene();
        set(txtStatus, 'String', 'Status: scene loaded', 'ForegroundColor', [0 0.45 0]);
    end

    function onPreview(~, ~)
        if ~state.loaded
            set(txtStatus, 'String', 'Status: load scene first', 'ForegroundColor', [0.85 0.2 0.1]);
            return;
        end

        features = state.perception.features;
        [pathBase, segments, keyPts, Tseq, fitInfo] = fit_three_segment_parabola( ...
            features, trajParams, postureParams, nPts); %#ok<ASGLU>
        state.pathBase = pathBase;
        state.T_target_seq = Tseq;
        state.keyPts = keyPts;

        renderBaseScene();
        hold(ax, 'on');
        plot3(ax, pathBase(:, 1), pathBase(:, 2), pathBase(:, 3), 'r-', 'LineWidth', 2.4);
        plot3(ax, keyPts.start(1), keyPts.start(2), keyPts.start(3), 'go', 'MarkerSize', 8, 'LineWidth', 1.8);
        plot3(ax, keyPts.deep(1), keyPts.deep(2), keyPts.deep(3), 'ko', 'MarkerSize', 8, 'LineWidth', 1.8);
        plot3(ax, keyPts.finish(1), keyPts.finish(2), keyPts.finish(3), 'bo', 'MarkerSize', 8, 'LineWidth', 1.8);

        set(txtStatus, 'String', 'Status: trajectory preview updated', 'ForegroundColor', [0 0.45 0]);
    end

    function onRun(~, ~)
        if ~state.loaded
            set(txtStatus, 'String', 'Status: load scene first', 'ForegroundColor', [0.85 0.2 0.1]);
            return;
        end

        set(txtStatus, 'String', 'Status: running planner...', 'ForegroundColor', [0 0.3 0.8]);
        drawnow;

        if isempty(state.pathBase) || isempty(state.T_target_seq)
            onPreview();
        end

        collisionOpts = struct('segmentSamples', 3, 'pointRadius', 0.018, 'verbose', false);
        collisionFcn = @(q) collision_policy_bucket( ...
            state.robot, q, state.robotInfo.baseFrame, state.bucketModel, collisionOpts);

        if collisionFcn(state.qCurrent)
            warning('[main.03] Current pose collides bucket. Falling back to home.');
            state.qCurrent = clamp_to_limits(homeConfiguration(state.robot), state.jointLimits);
            if collisionFcn(state.qCurrent)
                warning('[main.03] Home still collides. Keeping current and forcing degraded execution.');
            end
        end

        ikLocal = inverseKinematics('RigidBodyTree', state.robot);
        [qEntry, ~] = ikLocal(state.robotInfo.endEffector, state.T_target_seq(:, :, 1), [1 1 1 0.7 0.7 0.7], state.qCurrent);
        qEntry = clamp_to_limits(qEntry, state.jointLimits);
        qEntry = nearest_equivalent_to_ref(qEntry, state.qCurrent, state.jointLimits);

        plannerOpts = struct('maxIter', 2200, 'goalBias', 0.20, 'stepSize', 0.24, ...
            'nearRadius', 0.65, 'goalThresh', 0.20, 'edgeStep', 0.08, ...
            'shortcutIters', 100, 'verbose', true);
        try
            [qApproach, rrtInfo] = plan_rrtstar_joint_path(state.qCurrent, qEntry, state.jointLimits, collisionFcn, plannerOpts);
            fprintf('[main.03] approach success=%d nodes=%d pathPts=%d\n', ...
                rrtInfo.success, rrtInfo.numNodes, size(qApproach, 1));
        catch ME
            warning('[main.03] RRT* failed: %s', ME.message);
            qApproach = [state.qCurrent; qEntry];
            rrtInfo = struct('success', false, 'numNodes', 0, 'message', ME.message); %#ok<NASGU>
        end
        if size(qApproach, 1) < 2
            qApproach = [state.qCurrent; qEntry];
        end

        % Keep-complete strategy: if many hold points, rebuild trajectory with reduced depth.
        adaptiveTraj = trajParams;
        finalPath = state.pathBase;
        finalTseq = state.T_target_seq;
        dryInfo = struct('nHold', inf);
        for attempt = 1:4
            [pathA, ~, ~, TseqA, ~] = fit_three_segment_parabola( ...
                state.perception.features, adaptiveTraj, postureParams, nPts);
            dryOpts = struct( ...
                'render', false, ...
                'verbose', false, ...
                'prefixPath', qApproach, ...
                'bucketPointCloud', state.bucketPointCloud, ...
                'desiredPath', pathA, ...
                'localRrtEnabled', true, ...
                'localRrtMaxIter', 220);
            [~, dryInfo] = run_ik_and_animation( ...
                state.robot, state.robotInfo.endEffector, state.robotInfo.baseFrame, TseqA, ...
                qApproach(end, :), state.jointLimits, collisionFcn, dryOpts);

            fprintf('[main.03] adapt attempt=%d hold=%d fallback=%d posErrMax=%.4f\n', ...
                attempt, dryInfo.nHold, dryInfo.nFallback, dryInfo.posErrMax);

            finalPath = pathA;
            finalTseq = TseqA;
            if dryInfo.nHold <= 2
                break;
            end

            adaptiveTraj.entryDepthRatio = max(0.18, 0.93 * adaptiveTraj.entryDepthRatio);
            adaptiveTraj.deepDepthRatio = max(0.35, 0.90 * adaptiveTraj.deepDepthRatio);
            adaptiveTraj.exitDepthRatio = max(0.08, 0.92 * adaptiveTraj.exitDepthRatio);
            adaptiveTraj.entryLift = adaptiveTraj.entryLift + 0.01;
        end

        runOpts = struct( ...
            'render', true, ...
            'ax', ax, ...
            'verbose', true, ...
            'framePause', 0.020, ...
            'prefixPath', qApproach, ...
            'bucketPointCloud', state.bucketPointCloud, ...
            'desiredPath', finalPath, ...
            'localRrtEnabled', true, ...
            'localRrtMaxIter', 280);
        [qExec, runInfo] = run_ik_and_animation( ...
            state.robot, state.robotInfo.endEffector, state.robotInfo.baseFrame, finalTseq, ...
            qApproach(end, :), state.jointLimits, collisionFcn, runOpts);

        state.qCurrent = qExec(end, :);
        state.pathBase = finalPath;
        state.T_target_seq = finalTseq;

        set(txtStatus, 'String', sprintf('Status: done (hold=%d fallback=%d)', runInfo.nHold, runInfo.nFallback), ...
            'ForegroundColor', [0 0.45 0]);
    end

    function renderBaseScene()
        cla(ax);
        show(state.robot, state.qCurrent, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);
        hold(ax, 'on');
        pcshow(state.bucketPointCloud, 'Parent', ax, 'MarkerSize', 35);
        drawBucketWire(ax, state.bucketModel);
        title(ax, 'App3: bucket-aware planning and execution');
        grid(ax, 'on'); axis(ax, 'equal');
    end
end

function drawBucketWire(ax, b)
t = linspace(0, 2*pi, 120);
xt = b.topCenter(1) + (b.topRadius + b.wallThickness/2) * cos(t);
yt = b.topCenter(2) + (b.topRadius + b.wallThickness/2) * sin(t);
zt = b.topCenter(3) * ones(size(t));
xb = b.topCenter(1) + (b.bottomRadius + b.wallThickness/2) * cos(t);
yb = b.topCenter(2) + (b.bottomRadius + b.wallThickness/2) * sin(t);
zb = (b.topCenter(3) - b.depth) * ones(size(t));
plot3(ax, xt, yt, zt, 'm-', 'LineWidth', 1.8);
plot3(ax, xb, yb, zb, 'm-', 'LineWidth', 1.8);
for k = 1:8
    id = round(1 + (k-1) * (numel(t)-1) / 8);
    plot3(ax, [xt(id), xb(id)], [yt(id), yb(id)], [zt(id), zb(id)], 'm-', 'LineWidth', 1.2);
end
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
fprintf('[main.03] whitelist bodies: %s\n', strjoin(names, ', '));
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
