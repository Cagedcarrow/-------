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
params.usePaperDpRrt = true;
params.compareWithBaseline = true;
params.reverseFitZAxis = true;
params.savePreviewVideo = true;
params.stlNTheta = 120;
params.stlNHeight = 44;
params.dpGoalBiasInit = 0.28;
params.dpGoalBiasMin = 0.03;
params.dpRhoInit = 0.50;
params.dpDecayRate = 0.50;
params.dpLambdaMax = 0.24;
params.dpLambdaMin = 0.08;
params.dpDSafe = 0.35;
params.dpStepKappa = 3.00;
params.dpFailRecovery = 2;
params.dpMaxCollisionSamples = 1500;

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
cbPaperDp = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [panelX, 0.30, panelW, 0.03], 'String', 'use paper DP-RRT*', ...
    'Value', params.usePaperDpRrt, 'BackgroundColor', get(fig, 'Color'), ...
    'Callback', @(src,~) onToggleBool('usePaperDpRrt', src.Value));
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
            'forceGPU', logical(params.forceGPU), ...
            'goalBiasInit', params.dpGoalBiasInit, ...
            'goalBiasMin', params.dpGoalBiasMin, ...
            'rhoInit', params.dpRhoInit, ...
            'decayRate', params.dpDecayRate, ...
            'lambdaMax', params.dpLambdaMax, ...
            'lambdaMin', params.dpLambdaMin, ...
            'dSafe', params.dpDSafe, ...
            'stepKappa', params.dpStepKappa, ...
            'failRecovery', params.dpFailRecovery, ...
            'maxCollisionSamples', params.dpMaxCollisionSamples);

        if params.usePaperDpRrt
            primaryPlanner = 'dp_rrtstar_cuda';
            comparePlanner = 'rrtstar_cuda';
        else
            primaryPlanner = 'rrtstar_cuda';
            comparePlanner = 'dp_rrtstar_cuda';
        end
        fprintf('[main.05] planner primary=%s compare=%d baseline=%s\n', ...
            primaryPlanner, params.compareWithBaseline, comparePlanner);

        planSeed = max(1, floor(mod(now * 86400000, 2^31 - 1)));
        rng(planSeed, 'twister');
        [qApproach, rrtInfo] = runApproachPlanner(primaryPlanner, ...
            state.qCurrent, qEntry, state.jointLimits, collisionFcn, plannerOpts);
        rrtInfo.planSeed = planSeed;

        compareInfo = struct();
        if params.compareWithBaseline
            rng(planSeed, 'twister');
            [~, compareInfo] = runApproachPlanner(comparePlanner, ...
                state.qCurrent, qEntry, state.jointLimits, collisionFcn, plannerOpts);
            primarySuccess = getNumericField(rrtInfo, 'success', 0) > 0.5;
            baseSuccess = getNumericField(compareInfo, 'success', 0) > 0.5;
            primaryLen = getNumericField(rrtInfo, 'pathLength', nan);
            baseLen = getNumericField(compareInfo, 'pathLength', nan);
            primaryTime = getNumericField(rrtInfo, 'elapsedSec', nan);
            baseTime = getNumericField(compareInfo, 'elapsedSec', nan);
            fprintf(['[main.05][compare] primary=%s success=%d nodes=%d len=%.3f t=%.3fs | ' ...
                'baseline=%s success=%d nodes=%d len=%.3f t=%.3fs\n'], ...
                getTextField(rrtInfo, 'algorithm', primaryPlanner), getNumericField(rrtInfo, 'success', 0), ...
                getNumericField(rrtInfo, 'numNodes', 0), getNumericField(rrtInfo, 'pathLength', nan), ...
                getNumericField(rrtInfo, 'elapsedSec', nan), ...
                getTextField(compareInfo, 'algorithm', comparePlanner), getNumericField(compareInfo, 'success', 0), ...
                getNumericField(compareInfo, 'numNodes', 0), getNumericField(compareInfo, 'pathLength', nan), ...
                getNumericField(compareInfo, 'elapsedSec', nan));
            fprintf('[main.05][compare] delta(pathLen baseline-primary)=%.3f delta(time baseline-primary)=%.3f\n', ...
                getNumericField(compareInfo, 'pathLength', nan) - getNumericField(rrtInfo, 'pathLength', nan), ...
                getNumericField(compareInfo, 'elapsedSec', nan) - getNumericField(rrtInfo, 'elapsedSec', nan));
            if primarySuccess && baseSuccess && isfinite(primaryLen) && isfinite(baseLen)
                if primaryLen < baseLen
                    fprintf('[main.05][compare] quality: primary path is shorter by %.3f\n', baseLen - primaryLen);
                else
                    fprintf('[main.05][compare] quality: baseline path is shorter by %.3f\n', primaryLen - baseLen);
                end
            end
            if primarySuccess && baseSuccess && isfinite(primaryTime) && isfinite(baseTime)
                if primaryTime < baseTime
                    fprintf('[main.05][compare] speed: primary is faster by %.3fs\n', baseTime - primaryTime);
                else
                    fprintf('[main.05][compare] speed: baseline is faster by %.3fs\n', primaryTime - baseTime);
                end
            end
            if primarySuccess && ~baseSuccess
                fprintf('[main.05][compare] robustness: primary succeeded while baseline failed.\n');
            elseif ~primarySuccess && baseSuccess
                fprintf('[main.05][compare] robustness: baseline succeeded while primary failed.\n');
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
        summary.compareInfo = compareInfo;
        summary.primaryPlanner = getTextField(rrtInfo, 'algorithm', '');
        summary.comparePlannerEnabled = logical(params.compareWithBaseline);
        summary.compareStats = buildCompareStats(rrtInfo, compareInfo);
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

    function [qPlan, planInfo] = runApproachPlanner(mode, qStartIn, qGoalIn, jointLimitsIn, collisionFcnIn, plannerOptsIn)
        qPlan = [qStartIn; qGoalIn];
        planInfo = struct('success', false, 'numNodes', 0, 'pathLength', nan, ...
            'numPathPoints', size(qPlan, 1), 'algorithm', mode, 'message', 'not started');
        tPlan = tic;

        try
            switch lower(mode)
                case 'dp_rrtstar_cuda'
                    [qPlan, planInfo] = plan_dprrtstar_joint_path_cuda( ...
                        qStartIn, qGoalIn, jointLimitsIn, collisionFcnIn, plannerOptsIn);
                case 'rrtstar_cuda'
                    [qPlan, planInfo] = plan_rrtstar_joint_path_cuda( ...
                        qStartIn, qGoalIn, jointLimitsIn, collisionFcnIn, plannerOptsIn);
                case 'rrtstar_cpu'
                    [qPlan, planInfo] = plan_rrtstar_joint_path( ...
                        qStartIn, qGoalIn, jointLimitsIn, collisionFcnIn, ...
                        rmfield(plannerOptsIn, {'useGPU', 'forceGPU'}));
                otherwise
                    error('[main.05] unknown planner mode: %s', mode);
            end
        catch ME
            warning('[main.05] planner %s failed: %s', mode, ME.message);
            if strcmpi(mode, 'rrtstar_cuda') && ~plannerOptsIn.forceGPU
                fprintf('[main.05] fallback: rrtstar cpu\n');
                [qPlan, planInfo] = plan_rrtstar_joint_path( ...
                    qStartIn, qGoalIn, jointLimitsIn, collisionFcnIn, ...
                    rmfield(plannerOptsIn, {'useGPU', 'forceGPU'}));
                planInfo.algorithm = 'rrtstar_cpu_fallback';
                planInfo.gpu = struct('enabled', false, 'reason', 'fallback_cpu');
            elseif strcmpi(mode, 'dp_rrtstar_cuda') && ~plannerOptsIn.forceGPU
                fprintf('[main.05] fallback: dp-rrt* cpu mode\n');
                dpCpuOpts = plannerOptsIn;
                dpCpuOpts.useGPU = false;
                dpCpuOpts.forceGPU = false;
                [qPlan, planInfo] = plan_dprrtstar_joint_path_cuda( ...
                    qStartIn, qGoalIn, jointLimitsIn, collisionFcnIn, dpCpuOpts);
                planInfo.algorithm = 'dp_rrtstar_cpu_fallback';
                if ~isfield(planInfo, 'gpu')
                    planInfo.gpu = struct('enabled', false, 'reason', 'fallback_cpu');
                else
                    planInfo.gpu.enabled = false;
                    planInfo.gpu.reason = 'fallback_cpu';
                end
            else
                qPlan = [qStartIn; qGoalIn];
                planInfo = struct('success', false, 'numNodes', 0, ...
                    'pathLength', norm(qGoalIn - qStartIn), 'numPathPoints', size(qPlan, 1), ...
                    'algorithm', [mode '_failed'], 'message', ME.message, ...
                    'gpu', struct('enabled', false, 'reason', 'exception'));
            end
        end

        planInfo.elapsedSec = toc(tPlan);
        if ~isfield(planInfo, 'algorithm') || isempty(planInfo.algorithm)
            planInfo.algorithm = mode;
        end
        if ~isfield(planInfo, 'pathLength')
            planInfo.pathLength = pathLength(qPlan);
        end
        if ~isfield(planInfo, 'numPathPoints')
            planInfo.numPathPoints = size(qPlan, 1);
        end
        if ~isfield(planInfo, 'numNodes')
            planInfo.numNodes = 0;
        end
        fprintf('[main.05] planner %s done: success=%d nodes=%d pts=%d len=%.3f t=%.3fs\n', ...
            planInfo.algorithm, getNumericField(planInfo, 'success', 0), ...
            getNumericField(planInfo, 'numNodes', 0), size(qPlan, 1), ...
            getNumericField(planInfo, 'pathLength', nan), getNumericField(planInfo, 'elapsedSec', nan));
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

function v = getNumericField(s, fieldName, defaultValue)
if nargin < 3
    defaultValue = nan;
end
v = defaultValue;
if ~isstruct(s) || ~isfield(s, fieldName)
    return;
end
raw = s.(fieldName);
if isempty(raw)
    return;
end
if islogical(raw)
    v = double(raw);
    return;
end
if isnumeric(raw) && isscalar(raw)
    v = double(raw);
end
end

function t = getTextField(s, fieldName, defaultValue)
if nargin < 3
    defaultValue = '';
end
t = defaultValue;
if ~isstruct(s) || ~isfield(s, fieldName)
    return;
end
raw = s.(fieldName);
if isstring(raw) || ischar(raw)
    t = char(raw);
end
end

function L = pathLength(qPath)
if isempty(qPath) || size(qPath, 1) < 2
    L = 0;
    return;
end
d = diff(qPath, 1, 1);
L = sum(vecnorm(d, 2, 2));
end

function out = buildCompareStats(primaryInfo, baselineInfo)
out = struct();
out.primaryAlgorithm = getTextField(primaryInfo, 'algorithm', '');
out.baselineAlgorithm = getTextField(baselineInfo, 'algorithm', '');
out.primarySuccess = getNumericField(primaryInfo, 'success', 0) > 0.5;
out.baselineSuccess = getNumericField(baselineInfo, 'success', 0) > 0.5;
out.primaryPathLength = getNumericField(primaryInfo, 'pathLength', nan);
out.baselinePathLength = getNumericField(baselineInfo, 'pathLength', nan);
out.primaryTimeSec = getNumericField(primaryInfo, 'elapsedSec', nan);
out.baselineTimeSec = getNumericField(baselineInfo, 'elapsedSec', nan);
out.deltaPathLength = out.baselinePathLength - out.primaryPathLength;
out.deltaTimeSec = out.baselineTimeSec - out.primaryTimeSec;
if out.primarySuccess && ~out.baselineSuccess
    out.verdict = 'primary_more_robust';
elseif ~out.primarySuccess && out.baselineSuccess
    out.verdict = 'baseline_more_robust';
elseif out.primarySuccess && out.baselineSuccess
    if isfinite(out.deltaPathLength) && isfinite(out.deltaTimeSec)
        if out.deltaPathLength >= 0 && out.deltaTimeSec >= 0
            out.verdict = 'primary_better_both';
        elseif out.deltaPathLength < 0 && out.deltaTimeSec < 0
            out.verdict = 'baseline_better_both';
        else
            out.verdict = 'tradeoff';
        end
    else
        out.verdict = 'tradeoff';
    end
else
    out.verdict = 'both_failed';
end
end
