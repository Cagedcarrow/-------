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
trajParams.entrySpanFactor = 1.00;
trajParams.midSpanFactor = 0.35;
trajParams.exitSpanFactor = 1.00;
trajParams.entryLift = 0.04;
trajParams.entryDepthRatio = 0.20;
trajParams.deepDepthRatio = 0.25;
trajParams.exitDepthRatio = 0.16;
trajParams.midSlopeAtKnot1 = -0.20;
trajParams.localYOffset = 0.0;
trajParams.maxCutAngleDeg = 30.0;
trajParams.maxDeepDepthRatio = 0.25;
trajParams.minDeltaAroundDeep = 0.04;
trajParams.topOuterOffsetRatio = 0.125;
trajParams.startFinishLiftRatio = 0.25;
trajParams.targetDeepDepthRatio = 0.15;
trajParams.execFirstMode = true;
trajParams.forceBaseXDir = true;

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
sEntryD = createSliderControl(fig, panelX, 0.57, panelW, rowH, 'entryDepthRatio', 0.08, 0.35, trajParams.entryDepthRatio, ...
    @(v) onSetTraj('entryDepthRatio', v));
sDeepD = createSliderControl(fig, panelX, 0.50, panelW, rowH, 'deepDepth(固定0.15)', 0.12, 0.35, trajParams.deepDepthRatio, ...
    @(v) onSetTraj('deepDepthRatio', v));
set(sDeepD.slider, 'Enable', 'off');
set(sDeepD.valueText, 'String', '0.150');
sExitD = createSliderControl(fig, panelX, 0.43, panelW, rowH, 'exitDepthRatio', 0.05, 0.30, trajParams.exitDepthRatio, ...
    @(v) onSetTraj('exitDepthRatio', v));
sMidS = createSliderControl(fig, panelX, 0.36, panelW, rowH, 'midSlopeAtKnot1', -0.57, 0.30, trajParams.midSlopeAtKnot1, ...
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
        % Fixed by task requirement: deepest point at 0.15 bucket depth.
        trajParams.targetDeepDepthRatio = 0.15;

        features = state.perception.features;
        [pathBase, segments, keyPts, Tseq, fitInfo] = fit_three_segment_parabola( ...
            features, trajParams, postureParams, nPts); %#ok<ASGLU>
        state.pathBase = pathBase;
        state.T_target_seq = Tseq;
        state.keyPts = keyPts;
        deepAbs = features.topPoint(3) - keyPts.deep(3);
        startLift = keyPts.start(3) - features.topPoint(3);
        finishLift = keyPts.finish(3) - features.topPoint(3);
        edgeDist = fitInfo.outerOffset;
        fprintf(['[main.03] preview: deep=%.3fm (bucketDepth=%.3fm), ' ...
            'startLift=%.3fm finishLift=%.3fm outerOffset=%.3fm, ' ...
            'slopeMaxAfter=%.3f (<= tan30=%.3f), xAxis=%s\n'], ...
            deepAbs, features.depth, startLift, finishLift, edgeDist, fitInfo.slopeMaxAfter, tand(30), fitInfo.xAxisSource);

        renderBaseScene();
        hold(ax, 'on');
        % High-contrast preview curve (black base + yellow overlay) for visibility.
        plot3(ax, pathBase(:, 1), pathBase(:, 2), pathBase(:, 3), '-', ...
            'Color', [0.05 0.05 0.05], 'LineWidth', 5.0);
        plot3(ax, pathBase(:, 1), pathBase(:, 2), pathBase(:, 3), '-', ...
            'Color', [1.0 0.95 0.1], 'LineWidth', 2.6);
        scatter3(ax, pathBase(1:3:end, 1), pathBase(1:3:end, 2), pathBase(1:3:end, 3), ...
            22, [1.0 0.95 0.1], 'filled', 'MarkerEdgeColor', [0.1 0.1 0.1]);
        plot3(ax, keyPts.start(1), keyPts.start(2), keyPts.start(3), 'o', ...
            'MarkerSize', 10, 'MarkerFaceColor', [0 0.9 0], 'MarkerEdgeColor', [0 0.2 0], 'LineWidth', 1.8);
        plot3(ax, keyPts.deep(1), keyPts.deep(2), keyPts.deep(3), 'o', ...
            'MarkerSize', 10, 'MarkerFaceColor', [0.95 0.95 0.95], 'MarkerEdgeColor', [0.05 0.05 0.05], 'LineWidth', 1.8);
        plot3(ax, keyPts.finish(1), keyPts.finish(2), keyPts.finish(3), 'o', ...
            'MarkerSize', 10, 'MarkerFaceColor', [0.1 0.6 1], 'MarkerEdgeColor', [0 0.2 0.5], 'LineWidth', 1.8);
        text(ax, keyPts.start(1), keyPts.start(2), keyPts.start(3)+0.03, 'start', 'Color', [0 0.5 0], 'FontWeight', 'bold');
        text(ax, keyPts.deep(1), keyPts.deep(2), keyPts.deep(3)+0.03, 'deep', 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold');
        text(ax, keyPts.finish(1), keyPts.finish(2), keyPts.finish(3)+0.03, 'finish', 'Color', [0 0.3 0.7], 'FontWeight', 'bold');

        set(txtStatus, 'String', 'Status: trajectory preview updated', 'ForegroundColor', [0 0.45 0]);
    end

    function onRun(~, ~)
        if ~state.loaded
            set(txtStatus, 'String', 'Status: load scene first', 'ForegroundColor', [0.85 0.2 0.1]);
            return;
        end
        trajParams.targetDeepDepthRatio = 0.15;

        set(txtStatus, 'String', 'Status: running planner...', 'ForegroundColor', [0 0.3 0.8]);
        drawnow;
        fprintf('[main.03] ===== RUN START =====\n');
        fprintf('[main.03] current qNorm=%.3f\n', norm(state.qCurrent));

        if isempty(state.pathBase) || isempty(state.T_target_seq)
            fprintf('[main.03] no preview cache, building preview first...\n');
            onPreview();
        end
        fprintf('[main.03] preview points=%d\n', size(state.pathBase, 1));

        collisionOpts = struct('segmentSamples', 3, 'pointRadius', 0.018, 'verbose', false);
        collisionFcn = @(q) collision_policy_bucket( ...
            state.robot, q, state.robotInfo.baseFrame, state.bucketModel, collisionOpts);

        set(txtStatus, 'String', 'Status: checking start collision...', 'ForegroundColor', [0 0.3 0.8]);
        drawnow;
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
        fprintf('[main.03] qEntry solved, deltaNorm=%.3f\n', norm(qEntry - state.qCurrent));

        plannerOpts = struct('maxIter', 2200, 'goalBias', 0.20, 'stepSize', 0.24, ...
            'nearRadius', 0.65, 'goalThresh', 0.20, 'edgeStep', 0.08, ...
            'shortcutIters', 100, 'verbose', true, 'logEvery', 80);
        set(txtStatus, 'String', 'Status: RRT* approach planning...', 'ForegroundColor', [0 0.3 0.8]);
        drawnow;
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

        % Execution-first strategy: iterate presets that progressively relax geometry.
        finalPath = state.pathBase;
        finalTseq = state.T_target_seq;
        dryInfo = struct('nHold', inf);
        bestScore = inf;
        bestAttempt = 1;
        bestMetrics = struct();
        bestDryInfo = struct('timeoutTriggered', true, 'localRrtFailTotal', inf, ...
            'nSolved', 0, 'nHold', inf, 'nFallback', inf, 'posErrMax', inf);
        for attempt = 1:4
            [adaptiveTraj, nPtsTry] = buildExecPriorityAttempt(trajParams, attempt);
            set(txtStatus, 'String', sprintf('Status: adaptive check %d/4 ...', attempt), 'ForegroundColor', [0 0.3 0.8]);
            drawnow;
            fprintf(['[main.03] adaptive check attempt=%d | nPts=%d outerOffset=%.3f liftRatio=%.3f ' ...
                'targetDeep=%.3f maxCut=%.1f\n'], ...
                attempt, nPtsTry, adaptiveTraj.topOuterOffsetRatio, adaptiveTraj.startFinishLiftRatio, ...
                adaptiveTraj.targetDeepDepthRatio, adaptiveTraj.maxCutAngleDeg);
            [pathA, ~, ~, TseqA, ~] = fit_three_segment_parabola( ...
                state.perception.features, adaptiveTraj, postureParams, nPtsTry);
            dryOpts = struct( ...
                'render', false, ...
                'verbose', true, ...
                'prefixPath', qApproach, ...
                'bucketPointCloud', state.bucketPointCloud, ...
                'desiredPath', pathA, ...
                'localRrtEnabled', true, ...
                'localRrtMaxIter', 220, ...
                'maxConsecutiveLocalFail', 6, ...
                'maxTotalLocalFail', 18, ...
                'maxRunSeconds', 35, ...
                'maxIkSeconds', 14, ...
                'maxConnectSeconds', 20, ...
                'maxAnimSeconds', 5, ...
                'maxConsecutiveNoProgress', 10, ...
                'progressFcn', @onRunProgress);
            [~, dryInfo] = run_ik_and_animation( ...
                state.robot, state.robotInfo.endEffector, state.robotInfo.baseFrame, TseqA, ...
                qApproach(end, :), state.jointLimits, collisionFcn, dryOpts);

            [thisScore, metrics] = scoreAttemptForExecution(dryInfo, nPtsTry);
            fprintf(['[main.03] adapt attempt=%d solved=%d/%d ratio=%.3f hold=%d fallback=%d ' ...
                'timeout=%d localFail=%d posErrMax=%.4f score=%.2f\n'], ...
                attempt, dryInfo.nSolved, nPtsTry, metrics.solveRatio, dryInfo.nHold, dryInfo.nFallback, ...
                dryInfo.timeoutTriggered, dryInfo.localRrtFailTotal, safeNumber(dryInfo.posErrMax, nan), thisScore);

            if thisScore < bestScore
                bestScore = thisScore;
                finalPath = pathA;
                finalTseq = TseqA;
                bestAttempt = attempt;
                bestMetrics = metrics;
                bestDryInfo = dryInfo;
            end

            if (dryInfo.nHold <= 2) && (~dryInfo.timeoutTriggered)
                break;
            end
        end
        fprintf(['[main.03] selected attempt=%d score=%.2f solvedRatio=%.3f hold=%d fallback=%d ' ...
            'timeout=%d localFail=%d\n'], ...
            bestAttempt, bestScore, safeNumber(bestMetrics.solveRatio, 0), ...
            safeNumber(bestDryInfo.nHold, -1), safeNumber(bestDryInfo.nFallback, -1), ...
            safeNumber(bestDryInfo.timeoutTriggered, 1), safeNumber(bestDryInfo.localRrtFailTotal, -1));

        enableLocalRrtFinal = true;
        if bestDryInfo.timeoutTriggered || (safeNumber(bestDryInfo.localRrtFailTotal, 0) >= 8)
            enableLocalRrtFinal = false;
        end
        fprintf('[main.03] final policy: localRRT=%d (dryTimeout=%d localFail=%d)\n', ...
            enableLocalRrtFinal, safeNumber(bestDryInfo.timeoutTriggered, 0), safeNumber(bestDryInfo.localRrtFailTotal, 0));

        runOpts = struct( ...
            'render', true, ...
            'ax', ax, ...
            'verbose', true, ...
            'framePause', 0.020, ...
            'prefixPath', qApproach, ...
            'bucketPointCloud', state.bucketPointCloud, ...
            'desiredPath', finalPath, ...
            'localRrtEnabled', enableLocalRrtFinal, ...
            'localRrtMaxIter', 280, ...
            'maxConsecutiveLocalFail', 8, ...
            'maxTotalLocalFail', 30, ...
            'maxRunSeconds', 90, ...
            'maxIkSeconds', 35, ...
            'maxConnectSeconds', 95, ...
            'maxAnimSeconds', 180, ...
            'maxConsecutiveNoProgress', 18, ...
            'progressFcn', @onRunProgress);
        set(txtStatus, 'String', 'Status: executing IK + animation...', 'ForegroundColor', [0 0.3 0.8]);
        drawnow;
        fprintf('[main.03] execution begin, approachPts=%d targetPts=%d\n', size(qApproach,1), size(finalTseq,3));
        [qExec, runInfo] = run_ik_and_animation( ...
            state.robot, state.robotInfo.endEffector, state.robotInfo.baseFrame, finalTseq, ...
            qApproach(end, :), state.jointLimits, collisionFcn, runOpts);

        state.qCurrent = qExec(end, :);
        state.pathBase = finalPath;
        state.T_target_seq = finalTseq;

        if runInfo.timeoutTriggered
            stopStage = 'unknown';
            if isfield(runInfo, 'timeoutStage') && ~isempty(runInfo.timeoutStage)
                stopStage = runInfo.timeoutStage;
            end
            set(txtStatus, 'String', sprintf('Status: timeout stop[%s] (hold=%d, solved=%d)', stopStage, runInfo.nHold, runInfo.nSolved), ...
                'ForegroundColor', [0.85 0.35 0.0]);
        else
            set(txtStatus, 'String', sprintf('Status: done (hold=%d fallback=%d)', runInfo.nHold, runInfo.nFallback), ...
                'ForegroundColor', [0 0.45 0]);
        end
        fprintf('[main.03] ===== RUN DONE ===== hold=%d fallback=%d execPts=%d\n', ...
            runInfo.nHold, runInfo.nFallback, runInfo.nExecuted);
    end

    function onRunProgress(s)
        if ~isstruct(s) || ~isfield(s, 'message')
            return;
        end
        stage = '';
        if isfield(s, 'stage')
            stage = string(s.stage);
        end
        if stage == "timeout"
            set(txtStatus, 'String', ['Status: ', char(s.message)], 'ForegroundColor', [0.90 0.25 0.10]);
        elseif stage == "local_rrt_disabled"
            set(txtStatus, 'String', ['Status: ', char(s.message)], 'ForegroundColor', [0.85 0.45 0.05]);
        else
            set(txtStatus, 'String', ['Status: ', char(s.message)], 'ForegroundColor', [0 0.3 0.8]);
        end
        drawnow limitrate;
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

function [p, nPtsTry] = buildExecPriorityAttempt(baseParams, attempt)
% buildExecPriorityAttempt:
% Attempt presets ordered from strict geometry to execution-priority.

p = baseParams;
nPtsTry = 90;

switch attempt
    case 1
        p.execFirstMode = true;
        p.forceBaseXDir = true;
        p.maxCutAngleDeg = 30;
        nPtsTry = 90;
    case 2
        p.execFirstMode = true;
        p.forceBaseXDir = true;
        p.maxCutAngleDeg = 26;
        p.topOuterOffsetRatio = 0.08;
        p.startFinishLiftRatio = 0.22;
        p.midSlopeAtKnot1 = -0.16;
        nPtsTry = 80;
    case 3
        p.execFirstMode = true;
        p.forceBaseXDir = true;
        p.maxCutAngleDeg = 22;
        p.topOuterOffsetRatio = 0.04;
        p.startFinishLiftRatio = 0.18;
        p.targetDeepDepthRatio = 0.13;
        p.midSlopeAtKnot1 = -0.12;
        nPtsTry = 70;
    otherwise
        p.execFirstMode = true;
        p.forceBaseXDir = true;
        p.maxCutAngleDeg = 18;
        p.topOuterOffsetRatio = 0.00;
        p.startFinishLiftRatio = 0.14;
        p.targetDeepDepthRatio = 0.10;
        p.midSlopeAtKnot1 = -0.08;
        p.entryDepthRatio = min(p.entryDepthRatio, 0.18);
        p.exitDepthRatio = min(p.exitDepthRatio, 0.12);
        nPtsTry = 60;
end
end

function v = safeNumber(x, fallback)
if isempty(x) || ~isfinite(x)
    v = fallback;
else
    v = x;
end
end

function [score, metrics] = scoreAttemptForExecution(dryInfo, nPtsTry)
nPtsTry = max(1, nPtsTry);
solveRatio = safeNumber(dryInfo.nSolved, 0) / nPtsTry;
solveRatio = min(max(solveRatio, 0), 1);
timeoutPenalty = 0;
if isfield(dryInfo, 'timeoutTriggered') && dryInfo.timeoutTriggered
    timeoutPenalty = 250;
end
holdN = safeNumber(dryInfo.nHold, nPtsTry);
fallbackN = safeNumber(dryInfo.nFallback, nPtsTry);
localFail = safeNumber(dryInfo.localRrtFailTotal, 0);
posErr = safeNumber(dryInfo.posErrMax, 0.6);

% Execution-first scoring: prefer higher solved ratio and stable connect behavior.
score = timeoutPenalty ...
    + (1 - solveRatio) * 180 ...
    + holdN * 18 ...
    + fallbackN * 1.2 ...
    + localFail * 0.8 ...
    + posErr * 8;

metrics = struct();
metrics.solveRatio = solveRatio;
metrics.timeoutPenalty = timeoutPenalty;
end
