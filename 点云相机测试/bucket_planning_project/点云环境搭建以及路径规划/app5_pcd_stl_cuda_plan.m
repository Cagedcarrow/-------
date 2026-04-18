pcd_stl_cuda_plan_main();

function pcd_stl_cuda_plan_main()
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
paths.bucketStl = fullfile(ctx.dataDir, 'bucket_scene_from_pcd.stl');
paths.outMat = fullfile(ctx.dataDir, 'app5_gpu_plan_result.mat');
paths.videoRoot = fullfile(ctx.dataDir, 'video');

state = struct();
state.loaded = false;
state.sceneCfg = struct();
state.perception = struct();
state.features = struct();
state.robot = [];
state.jointLimits = [];
state.qCurrent = [];
state.robotInfo = struct();
state.bucketPointCloud = [];
state.bucketModel = struct();
state.stlInfo = struct('vertices', [], 'faces', [], 'stlSaved', false);
state.pathBase = [];
state.T_target_seq = [];
state.keyPts = struct();
state.fitInfo = struct();
state.fitVideoPath = '';
state.execVideoPath = '';
state.videoSessionDir = '';

params = struct();
params.attackDeg = -12.0;
params.assemblyDeg = 0.0;
params.flipToolZ = true;
params.targetDeepDepthRatio = 0.15;
params.entryDepthRatio = 0.20;
params.maxCutAngleDeg = 30.0;
params.midSlopeAtKnot1 = -0.20;
params.topOuterOffsetRatio = 0.125;
params.startFinishLiftRatio = 0.25;
params.nPts = 90;
params.useGPU = true;
params.forceGPU = false;
params.reverseFitZAxis = true;
params.savePreviewVideo = true;
params.stlNTheta = 120;
params.stlNHeight = 44;

fig = figure('Name', 'App5 - PCD -> STL + CUDA RRT* Planner', 'Color', 'w', ...
    'Position', [90, 50, 1560, 860]);
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.04, 0.08, 0.72, 0.88]);
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
view(ax, 132, 24);
xlim(ax, [-2.2, 2.2]);
ylim(ax, [-2.2, 2.2]);
zlim(ax, [-0.3, 2.5]);

panelX = 0.79;
panelW = 0.19;
rowH = 0.060;

uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.93, panelW, 0.045], 'String', 'Load + Build STL', ...
    'FontWeight', 'bold', 'Callback', @onLoadBuild);
uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.88, panelW, 0.045], 'String', 'Preview Path', ...
    'Callback', @onPreviewButton);
uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [panelX, 0.83, panelW, 0.045], 'String', 'Plan && Execute (CUDA)', ...
    'FontWeight', 'bold', 'Callback', @onRun);

sAttack = createSliderControl(fig, panelX, 0.75, panelW, rowH, 'attackDeg', -45, 45, params.attackDeg, ...
    @(v) onSet('attackDeg', v), '%.1f');
sAssembly = createSliderControl(fig, panelX, 0.68, panelW, rowH, 'assemblyDeg', -180, 180, params.assemblyDeg, ...
    @(v) onSet('assemblyDeg', v), '%.1f');
sDeep = createSliderControl(fig, panelX, 0.61, panelW, rowH, 'targetDeepDepthRatio', 0.08, 0.30, params.targetDeepDepthRatio, ...
    @(v) onSet('targetDeepDepthRatio', v), '%.3f');
sEntry = createSliderControl(fig, panelX, 0.54, panelW, rowH, 'entryDepthRatio', 0.08, 0.35, params.entryDepthRatio, ...
    @(v) onSet('entryDepthRatio', v), '%.3f');
sCut = createSliderControl(fig, panelX, 0.47, panelW, rowH, 'maxCutAngleDeg', 10, 45, params.maxCutAngleDeg, ...
    @(v) onSet('maxCutAngleDeg', v), '%.1f');
sOuter = createSliderControl(fig, panelX, 0.40, panelW, rowH, 'topOuterOffsetRatio', 0.00, 0.30, params.topOuterOffsetRatio, ...
    @(v) onSet('topOuterOffsetRatio', v), '%.3f');
sNPts = createSliderControl(fig, panelX, 0.33, panelW, rowH, 'nPts', 50, 180, params.nPts, ...
    @(v) onSetNPts(v), '%.0f');

cbFlip = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [panelX, 0.27, panelW, 0.03], 'String', 'flipToolZ', ...
    'Value', params.flipToolZ, 'BackgroundColor', get(fig, 'Color'), ...
    'Callback', @(src,~) onToggleBool('flipToolZ', src.Value));
cbReverseZ = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [panelX, 0.24, panelW, 0.03], 'String', 'reverseFitZAxis', ...
    'Value', params.reverseFitZAxis, 'BackgroundColor', get(fig, 'Color'), ...
    'Callback', @(src,~) onToggleBool('reverseFitZAxis', src.Value));
cbGPU = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [panelX, 0.21, panelW, 0.03], 'String', 'useGPU (CUDA)', ...
    'Value', params.useGPU, 'BackgroundColor', get(fig, 'Color'), ...
    'Callback', @(src,~) onToggleBool('useGPU', src.Value));
cbForceGPU = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [panelX, 0.18, panelW, 0.03], 'String', 'forceGPU (strict)', ...
    'Value', params.forceGPU, 'BackgroundColor', get(fig, 'Color'), ...
    'Callback', @(src,~) onToggleBool('forceGPU', src.Value));
cbSaveVideo = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [panelX, 0.15, panelW, 0.03], 'String', 'save preview video', ...
    'Value', params.savePreviewVideo, 'BackgroundColor', get(fig, 'Color'), ...
    'Callback', @(src,~) onToggleBool('savePreviewVideo', src.Value));

txtStatus = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [panelX, 0.11, panelW, 0.04], 'String', 'Status: waiting load', ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(fig, 'Color'), ...
    'ForegroundColor', [0 0.45 0], 'FontSize', 10);

txtLog = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [panelX, 0.01, panelW, 0.10], ...
    'String', sprintf('pcd:\n%s\nstl:\n%s', paths.bucketPcd, paths.bucketStl), ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(fig, 'Color'), ...
    'ForegroundColor', [0.2 0.2 0.2], 'FontSize', 9);

    function onSet(name, value)
        params.(name) = value;
        if state.loaded
            previewCore(false);
        end
    end

    function onSetInt(name, value, sld, txt)
        v = round(value);
        sld.Value = v;
        txt.String = sprintf('%.0f', v);
        params.(name) = v;
        if state.loaded
            previewCore(false);
        end
    end

    function onSetNPts(value)
        onSetInt('nPts', value, sNPts.slider, sNPts.valueText);
    end

    function onToggleBool(name, value)
        params.(name) = logical(value);
        if state.loaded && (strcmp(name, 'flipToolZ') || strcmp(name, 'reverseFitZAxis'))
            previewCore(false);
        end
    end

    function onLoadBuild(~, ~)
        setStatus('Status: loading scene + building STL...', [0 0.3 0.8]);
        fprintf('[main.05] ===== START =====\n');

        if ~isfile(paths.sceneCfg)
            error('[main.05] Missing %s. Run main/app1_environment_setup.m first.', paths.sceneCfg);
        end

        sceneCfg = load(paths.sceneCfg);
        if isfield(sceneCfg, 'pcdFile') && isfile(sceneCfg.pcdFile)
            pcdPath = sceneCfg.pcdFile;
        else
            pcdPath = paths.bucketPcd;
        end
        if ~isfile(pcdPath)
            error('[main.05] Missing PCD file: %s', pcdPath);
        end

        perception = struct();
        if isfile(paths.perception)
            perception = load(paths.perception);
        end

        if isfield(perception, 'features') && ~isempty(perception.features)
            features = perception.features;
            fprintf('[main.05] use cached perception features.\n');
        else
            fprintf('[main.05] perception cache missing, estimate from pcd...\n');
            features = estimate_bucket_from_pcd(pcdPath, struct('gridStep', 0.006));
        end

        wallThickness = 0.02;
        if isfield(perception, 'wallThickness') && ~isempty(perception.wallThickness)
            wallThickness = perception.wallThickness;
        elseif isfield(sceneCfg, 'bucketParams') && isfield(sceneCfg.bucketParams, 'wallThickness')
            wallThickness = sceneCfg.bucketParams.wallThickness;
        end

        stlInfo = build_bucket_stl_from_pcd(pcdPath, paths.bucketStl, struct( ...
            'features', features, ...
            'nTheta', params.stlNTheta, ...
            'nHeight', params.stlNHeight, ...
            'verbose', true));
        fprintf('[main.05] STL ready: %s (saved=%d)\n', paths.bucketStl, stlInfo.stlSaved);

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

        bucketModel = struct();
        bucketModel.topCenter = features.topPoint;
        bucketModel.topRadius = features.topRadius;
        bucketModel.bottomRadius = features.bottomRadius;
        bucketModel.depth = features.depth;
        bucketModel.wallThickness = wallThickness;
        bucketModel.whitelistBodies = inferToolWhitelist(robot);

        state.loaded = true;
        state.sceneCfg = sceneCfg;
        state.perception = perception;
        state.features = features;
        state.robot = robot;
        state.jointLimits = jointLimits;
        state.qCurrent = qHome;
        state.robotInfo = robotInfo;
        state.bucketPointCloud = pcread(pcdPath);
        state.bucketModel = bucketModel;
        state.stlInfo = stlInfo;
        state.pathBase = [];
        state.T_target_seq = [];
        state.keyPts = struct();
        state.fitInfo = struct();

        renderBaseScene();
        setStatus('Status: loaded + STL built', [0 0.45 0]);
    end

    function onPreviewButton(~, ~)
        previewCore(logical(params.savePreviewVideo), '', '');
    end

    function previewCore(saveVideo, targetDir, videoName)
        if nargin < 2
            targetDir = '';
        end
        if nargin < 3
            videoName = '';
        end
        if ~state.loaded
            setStatus('Status: load first', [0.85 0.2 0.1]);
            return;
        end

        trajParams = buildTrajParams(params);
        postureParams = buildPostureParams(params);
        [pathBase, ~, keyPts, Tseq, fitInfo] = fit_three_segment_parabola( ...
            state.features, trajParams, postureParams, params.nPts);

        state.pathBase = pathBase;
        state.T_target_seq = Tseq;
        state.keyPts = keyPts;
        state.fitInfo = fitInfo;

        renderBaseScene();
        hold(ax, 'on');
        plot3(ax, pathBase(:, 1), pathBase(:, 2), pathBase(:, 3), '-', 'Color', [0.35 0.00 0.00], 'LineWidth', 4.2);
        plot3(ax, pathBase(:, 1), pathBase(:, 2), pathBase(:, 3), '-', 'Color', [1.00 0.00 0.00], 'LineWidth', 2.6);
        plot3(ax, keyPts.start(1), keyPts.start(2), keyPts.start(3), 'go', 'MarkerSize', 8, 'LineWidth', 1.4);
        plot3(ax, keyPts.deep(1), keyPts.deep(2), keyPts.deep(3), 'ko', 'MarkerSize', 8, 'LineWidth', 1.4);
        plot3(ax, keyPts.finish(1), keyPts.finish(2), keyPts.finish(3), 'bo', 'MarkerSize', 8, 'LineWidth', 1.4);

        fprintf('[main.05] preview path points=%d deep=%.3f/%.3f slope=%.3f\n', ...
            size(pathBase, 1), fitInfo.deepAbs, fitInfo.deepTargetAbs, fitInfo.slopeMaxAfter);
        if saveVideo
            setStatus('Status: saving fit video...', [0 0.3 0.8]);
            state.fitVideoPath = saveFitProcessVideo(pathBase, keyPts, targetDir, videoName);
        end
        setStatus('Status: preview updated', [0 0.45 0]);
    end

    function onRun(~, ~)
        if ~state.loaded
            setStatus('Status: load first', [0.85 0.2 0.1]);
            return;
        end
        if ~isfolder(paths.videoRoot)
            mkdir(paths.videoRoot);
        end
        sessionTs = datestr(now, 'yyyymmdd_HHMMSS_FFF');
        sessionDir = fullfile(paths.videoRoot, sessionTs);
        mkdir(sessionDir);
        state.videoSessionDir = sessionDir;
        state.execVideoPath = '';
        fprintf('[main.05] video session dir: %s\n', sessionDir);

        setStatus('Status: preview + save fit video...', [0 0.3 0.8]);
        previewCore(true, sessionDir, '01_fit_process.mp4');
        if isempty(state.fitVideoPath)
            warning('[main.05] fit video save failed for session: %s', sessionDir);
        else
            fprintf('[main.05] fit video path: %s\n', state.fitVideoPath);
        end
        setStatus('Status: planning with CUDA...', [0 0.3 0.8]);

        collisionOpts = struct('segmentSamples', 3, 'pointRadius', 0.018, 'verbose', false);
        collisionFcn = @(q) collision_policy_bucket( ...
            state.robot, q, state.robotInfo.baseFrame, state.bucketModel, collisionOpts);

        if collisionFcn(state.qCurrent)
            warning('[main.05] current pose collides bucket. reset to home.');
            state.qCurrent = clamp_to_limits(homeConfiguration(state.robot), state.jointLimits);
        end

        ikLocal = inverseKinematics('RigidBodyTree', state.robot);
        [qEntry, ~] = ikLocal(state.robotInfo.endEffector, state.T_target_seq(:, :, 1), [1 1 1 0.7 0.7 0.7], state.qCurrent);
        qEntry = clamp_to_limits(qEntry, state.jointLimits);
        qEntry = nearest_equivalent_to_ref(qEntry, state.qCurrent, state.jointLimits);
        fprintf('[main.05] qEntry deltaNorm=%.3f\n', norm(qEntry - state.qCurrent));

        plannerOpts = struct( ...
            'maxIter', 2400, ...
            'goalBias', 0.22, ...
            'stepSize', 0.24, ...
            'nearRadius', 0.65, ...
            'goalThresh', 0.20, ...
            'edgeStep', 0.08, ...
            'shortcutIters', 110, ...
            'verbose', true, ...
            'logEvery', 80, ...
            'useGPU', logical(params.useGPU), ...
            'forceGPU', logical(params.forceGPU));

        try
            [qApproach, rrtInfo] = plan_rrtstar_joint_path_cuda( ...
                state.qCurrent, qEntry, state.jointLimits, collisionFcn, plannerOpts);
            fprintf('[main.05] approach success=%d nodes=%d pts=%d gpu=%d\n', ...
                rrtInfo.success, rrtInfo.numNodes, size(qApproach, 1), rrtInfo.gpu.enabled);
        catch ME
            warning('[main.05] CUDA planner failed: %s', ME.message);
            if ~params.forceGPU
                fprintf('[main.05] fallback to CPU rrtstar...\n');
                [qApproach, infoCPU] = plan_rrtstar_joint_path( ...
                    state.qCurrent, qEntry, state.jointLimits, collisionFcn, ...
                    rmfield(plannerOpts, {'useGPU', 'forceGPU'}));
                rrtInfo = infoCPU;
                rrtInfo.gpu = struct('enabled', false, 'reason', 'fallback_cpu');
            else
                qApproach = [state.qCurrent; qEntry];
                rrtInfo = struct('success', false, 'numNodes', 0, ...
                    'message', ME.message, 'gpu', struct('enabled', false, 'reason', 'force_gpu_exception'));
            end
        end
        if size(qApproach, 1) < 2
            qApproach = [state.qCurrent; qEntry];
        end

        setStatus('Status: executing trajectory...', [0 0.3 0.8]);
        runOpts = struct( ...
            'render', true, ...
            'ax', ax, ...
            'verbose', true, ...
            'framePause', 0.020, ...
            'prefixPath', qApproach, ...
            'bucketPointCloud', state.bucketPointCloud, ...
            'desiredPath', state.pathBase, ...
            'localRrtEnabled', true, ...
            'localRrtMaxIter', 260, ...
            'maxConsecutiveLocalFail', 8, ...
            'maxTotalLocalFail', 30, ...
            'maxRunSeconds', 120, ...
            'maxIkSeconds', 40, ...
            'maxConnectSeconds', 110, ...
            'maxAnimSeconds', 240, ...
            'maxConsecutiveNoProgress', 20, ...
            'saveVideo', true, ...
            'videoPath', fullfile(sessionDir, '02_robot_execution_fit.mp4'), ...
            'videoFrameRate', 24, ...
            'videoFrameStep', 1);

        [qExec, runInfo] = run_ik_and_animation( ...
            state.robot, state.robotInfo.endEffector, state.robotInfo.baseFrame, state.T_target_seq, ...
            qApproach(end, :), state.jointLimits, collisionFcn, runOpts);
        state.execVideoPath = runInfo.videoPath;
        if runInfo.videoSaved
            fprintf('[main.05] execution video path: %s\n', runInfo.videoPath);
        else
            warning('[main.05] execution video save failed, target=%s', runOpts.videoPath);
        end

        state.qCurrent = qExec(end, :);
        renderBaseScene();
        hold(ax, 'on');
        plot3(ax, state.pathBase(:, 1), state.pathBase(:, 2), state.pathBase(:, 3), '-', 'Color', [1.00 0.00 0.00], 'LineWidth', 2.6);

        summary = struct();
        summary.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
        summary.pcdPath = paths.bucketPcd;
        summary.stlPath = paths.bucketStl;
        summary.stlSaved = state.stlInfo.stlSaved;
        summary.features = state.features;
        summary.fitInfo = state.fitInfo;
        summary.rrtInfo = rrtInfo;
        summary.runInfo = runInfo;
        summary.params = params;
        summary.videoSessionDir = state.videoSessionDir;
        summary.fitVideoPath = state.fitVideoPath;
        summary.execVideoPath = state.execVideoPath;
        summary.execVideoSaved = runInfo.videoSaved;
        summary.qCurrent = state.qCurrent;
        summary.qEntry = qEntry;
        summary.qApproach = qApproach;
        summary.qExec = qExec;
        save(paths.outMat, '-struct', 'summary');
        fprintf('[main.05] saved result mat: %s\n', paths.outMat);

        if runInfo.timeoutTriggered
            setStatus(sprintf('Status: done with timeout [%s]', runInfo.timeoutStage), [0.88 0.35 0.05]);
        else
            setStatus('Status: done', [0 0.45 0]);
        end
        fprintf('[main.05] ===== DONE ===== timeout=%d hold=%d fallback=%d\n', ...
            runInfo.timeoutTriggered, runInfo.nHold, runInfo.nFallback);
        fprintf('[main.05] videos: fit=%s | exec=%s\n', summary.fitVideoPath, summary.execVideoPath);
    end

    function renderBaseScene()
        cla(ax);
        if isempty(state.robot)
            grid(ax, 'on'); axis(ax, 'equal');
            return;
        end
        show(state.robot, state.qCurrent, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);
        hold(ax, 'on');
        if ~isempty(state.bucketPointCloud)
            pcshow(state.bucketPointCloud, 'Parent', ax, 'MarkerSize', 35);
        end
        if ~isempty(state.stlInfo.vertices) && ~isempty(state.stlInfo.faces)
            drawStlMesh(ax, state.stlInfo.vertices, state.stlInfo.faces);
        end
        title(ax, 'App5: PCD->STL + CUDA planner');
        grid(ax, 'on');
        axis(ax, 'equal');
        drawnow limitrate;
    end

    function setStatus(textValue, colorValue)
        set(txtStatus, 'String', textValue, 'ForegroundColor', colorValue);
        drawnow limitrate;
    end

    function videoPath = saveFitProcessVideo(pathBase, keyPts, targetDir, videoName)
        if nargin < 3 || isempty(targetDir)
            if ~isfolder(paths.videoRoot)
                mkdir(paths.videoRoot);
            end
            tsDir = datestr(now, 'yyyymmdd_HHMMSS_FFF');
            targetDir = fullfile(paths.videoRoot, ['preview_', tsDir]);
        end
        if nargin < 4
            videoName = '';
        end
        if ~isfolder(targetDir)
            mkdir(targetDir);
        end
        if isempty(videoName)
            ts = datestr(now, 'yyyymmdd_HHMMSS_FFF');
            videoName = ['fit_process_', ts, '.mp4'];
        end
        videoPath = fullfile(targetDir, videoName);
        vw = VideoWriter(videoPath, 'MPEG-4');
        vw.FrameRate = 24;

        fprintf('[main.05] save fit video: %s\n', videoPath);
        open(vw);
        try
            cam = captureCameraState(ax);
            renderBaseScene();
            restoreCameraState(ax, cam);
            hold(ax, 'on');
            hLine = plot3(ax, pathBase(1, 1), pathBase(1, 2), pathBase(1, 3), '-', ...
                'Color', [1.00 0.00 0.00], 'LineWidth', 2.8);
            writeVideo(vw, getframe(fig));

            n = size(pathBase, 1);
            step = max(1, floor(n / 90));
            for i = 2:n
                hLine.XData = pathBase(1:i, 1);
                hLine.YData = pathBase(1:i, 2);
                hLine.ZData = pathBase(1:i, 3);
                if mod(i, step) == 0 || i == n
                    writeVideo(vw, getframe(fig));
                end
            end

            plot3(ax, keyPts.start(1), keyPts.start(2), keyPts.start(3), 'go', 'MarkerSize', 8, 'LineWidth', 1.4);
            plot3(ax, keyPts.deep(1), keyPts.deep(2), keyPts.deep(3), 'ko', 'MarkerSize', 8, 'LineWidth', 1.4);
            plot3(ax, keyPts.finish(1), keyPts.finish(2), keyPts.finish(3), 'bo', 'MarkerSize', 8, 'LineWidth', 1.4);
            for k = 1:10
                writeVideo(vw, getframe(fig));
            end
            close(vw);
            fprintf('[main.05] fit video saved ok\n');
        catch ME
            try
                close(vw);
            catch
            end
            warning('[main.05] save fit video failed: %s', ME.message);
            videoPath = '';
        end
    end
end

function trajParams = buildTrajParams(params)
trajParams = struct();
trajParams.entrySpanFactor = 1.00;
trajParams.midSpanFactor = 0.35;
trajParams.exitSpanFactor = 1.00;
trajParams.entryDepthRatio = params.entryDepthRatio;
trajParams.midSlopeAtKnot1 = params.midSlopeAtKnot1;
trajParams.localYOffset = 0.0;
trajParams.maxCutAngleDeg = params.maxCutAngleDeg;
trajParams.minDeltaAroundDeep = 0.04;
trajParams.topOuterOffsetRatio = params.topOuterOffsetRatio;
trajParams.startFinishLiftRatio = params.startFinishLiftRatio;
trajParams.targetDeepDepthRatio = params.targetDeepDepthRatio;
trajParams.execFirstMode = true;
trajParams.forceBaseXDir = true;
trajParams.reverseFitZAxis = logical(params.reverseFitZAxis);
end

function postureParams = buildPostureParams(params)
postureParams = struct();
postureParams.attackDeg = params.attackDeg;
postureParams.assemblyDeg = params.assemblyDeg;
postureParams.flipToolZ = logical(params.flipToolZ);
end

function drawStlMesh(ax, V, F)
patch('Parent', ax, 'Faces', F, 'Vertices', V, ...
    'FaceColor', [0.95 0.32 0.32], 'FaceAlpha', 0.14, ...
    'EdgeColor', [0.75 0.2 0.2], 'EdgeAlpha', 0.10);
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
fprintf('[main.05] whitelist bodies: %s\n', strjoin(names, ', '));
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

function out = createSliderControl(parentFig, px, py, pw, rh, label, vmin, vmax, v0, cb, fmt)
if nargin < 11 || isempty(fmt)
    fmt = '%.3f';
end

uicontrol('Parent', parentFig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [px, py, pw, rh * 0.45], 'String', label, ...
    'HorizontalAlignment', 'left', 'BackgroundColor', get(parentFig, 'Color'));

out.slider = uicontrol('Parent', parentFig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [px, py - rh * 0.45, pw * 0.78, rh * 0.45], ...
    'Min', vmin, 'Max', vmax, 'Value', v0, ...
    'Callback', @(src, ~) onChange(src));

out.valueText = uicontrol('Parent', parentFig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [px + pw * 0.80, py - rh * 0.45, pw * 0.20, rh * 0.45], ...
    'String', sprintf(fmt, v0), 'HorizontalAlignment', 'left', ...
    'BackgroundColor', get(parentFig, 'Color'));

    function onChange(src)
        val = src.Value;
        out.valueText.String = sprintf(fmt, val);
        cb(val);
    end
end
