function shovel_bucket_parabola_ik_demo_assembly_comp()
% UR10 shovel parabola scoop demo (hanging setup + GUI tune + run button)
% - Robot is mounted upside-down (lab setup)
% - Parabola is defined in robot base-link frame and can be tuned by GUI sliders
% - Tool orientation enforces: TCP y-axis aligns with parabola tangent direction
% - On RUN click: real-time command line logs for IK + animation progress

    clc;
    close all;

    fprintf('\n================ PARABOLA SCOOP IK DEMO ================\n');
    fprintf('[INFO] script=%s\n', mfilename('fullpath'));
    fprintf('[INFO] version=2026-04-17-parabola-v7-assembly-offset\n');

    %% 1) Model init
    urdfFile = 'ur10_world.urdf';
    baseFrame = 'world_base';
    baseLink = 'ur10';
    endEffector = 'shovel_tip';

    fprintf('[INFO] Loading URDF: %s\n', urdfFile);
    if ~exist(urdfFile, 'file')
        error('[ERROR] URDF file not found: %s', urdfFile);
    end

    robot = importrobot(urdfFile);
    robot.DataFormat = 'row';

    if ~any(strcmp(robot.BodyNames, endEffector))
        error('[ERROR] End effector "%s" not found.', endEffector);
    end
    if ~any(strcmp(robot.BodyNames, baseLink))
        error('[ERROR] Base link "%s" not found.', baseLink);
    end

    fprintf('[INFO] Robot loaded. NumBodies=%d, BaseName=%s\n', robot.NumBodies, robot.BaseName);

    %% 2) Hanging mount (upside-down)
    % Use same convention as your project: roll=180 deg in ZYX eul order.
    baseXYZ = [0, 0, 1.40];
    baseYPR = [0, 0, pi]; % [yaw pitch roll] rad, roll=pi => upside-down
    T_world_to_baseLink = trvec2tform(baseXYZ) * eul2tform(baseYPR, 'ZYX');

    baseBody = getBody(robot, baseLink);
    setFixedTransform(baseBody.Joint, T_world_to_baseLink);

    fprintf('[INFO] Hanging base applied. xyz=[%.3f %.3f %.3f], yprDeg=[%.1f %.1f %.1f]\n', ...
        baseXYZ, rad2deg(baseYPR));
    fprintf('[DEBUG] world_to_%s transform:\n', baseLink);
    disp(T_world_to_baseLink);

    ik = inverseKinematics('RigidBodyTree', robot);
    ikWeights = [1 1 1 0.6 0.6 0.6];
    jointLimits = collectJointLimits(robot);

    qHome = homeConfiguration(robot);
    qCurrent = clampToLimits(qHome, jointLimits);

    %% 3) Runtime state / params (trajectory in baseLink frame)
    nPts = 100;
    params = struct();
    params.centerX = 1.00;
    params.yConst = 0.00;
    params.zTop = 0.20;
    params.halfSpan = 0.20;
    params.depth = 0.50;
    params.pitchBiasDeg = -12.0;
    params.assemblyOffsetDeg = 0.0;
    params.framePause = 0.03;
    params.reachRadius = estimateReachRadius(robot, baseLink, endEffector, 0.20);
    params.showReachSphere = true;
    params.flipToolZ = true;

    fprintf('[INFO] Estimated reach sphere radius=%.3f m (with margin)\n', params.reachRadius);

    running = false;
    pathBase = [];
    pathWorld = [];
    coeff = struct('a',0,'b',0,'c',0);
    keyBase = struct('start',[],'vertex',[],'finish',[]);
    keyWorld = struct('start',[],'vertex',[],'finish',[]);

    %% 4) GUI
    fig = figure('Name', 'Parabola Scoop Debug GUI', 'Color', 'w', ...
        'Position', [80, 70, 1400, 820]);

    ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.05, 0.08, 0.67, 0.88]);
    grid(ax, 'on'); axis(ax, 'equal');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    view(ax, 130, 25);

    xRange = [-2.5, 2.5];
    yRange = [-2.5, 2.5];
    zRange = [-2.5, 2.5];
    xlim(ax, xRange); ylim(ax, yRange); zlim(ax, zRange);

    title(ax, '倒挂机械臂 + 抛物线轨迹预览 (相对基座坐标可调)');

    panelX = 0.75;
    panelW = 0.22;
    rowH = 0.075;
    [sphereX, sphereY, sphereZ] = sphere(28);

    sCenterX = createSliderControl(fig, panelX, 0.90, panelW, rowH, ...
        'centerX (base)', 0.50, 1.60, params.centerX, @(v) onParamChange('centerX', v));
    sYConst = createSliderControl(fig, panelX, 0.82, panelW, rowH, ...
        'yConst (base)', -0.60, 0.60, params.yConst, @(v) onParamChange('yConst', v));
    sZTop = createSliderControl(fig, panelX, 0.74, panelW, rowH, ...
        'zTop (base)', -0.20, 0.80, params.zTop, @(v) onParamChange('zTop', v));
    sHalfSpan = createSliderControl(fig, panelX, 0.66, panelW, rowH, ...
        'halfSpan (m)', 0.05, 0.60, params.halfSpan, @(v) onParamChange('halfSpan', v));
    sDepth = createSliderControl(fig, panelX, 0.58, panelW, rowH, ...
        'depth (m)', 0.05, 1.20, params.depth, @(v) onParamChange('depth', v));
    sPitchBias = createSliderControl(fig, panelX, 0.50, panelW, rowH, ...
        'pitchBias (deg)', -45, 45, params.pitchBiasDeg, @(v) onParamChange('pitchBiasDeg', v));
    sAssemblyOffset = createSliderControl(fig, panelX, 0.42, panelW, rowH, ...
        '装配偏角 (deg)', -180, 180, params.assemblyOffsetDeg, @(v) onParamChange('assemblyOffsetDeg', v));
    sSpeed = createSliderControl(fig, panelX, 0.34, panelW, rowH, ...
        'framePause (s)', 0.005, 0.10, params.framePause, @(v) onParamChange('framePause', v));
    sReach = createSliderControl(fig, panelX, 0.26, panelW, rowH, ...
        'reachRadius (m)', 0.60, 3.00, params.reachRadius, @(v) onParamChange('reachRadius', v));
    cbFlipZ = uicontrol('Parent', fig, 'Style', 'checkbox', 'Units', 'normalized', ...
        'Position', [panelX, 0.19, panelW, 0.03], 'String', '反向工具Z轴 (测试装反)', ...
        'Value', params.flipToolZ, 'BackgroundColor', get(fig,'Color'), ...
        'Callback', @onFlipZChanged);

    btnRun = uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [panelX, 0.12, panelW, 0.06], 'String', '运行 (Run)', ...
        'FontSize', 12, 'FontWeight', 'bold', 'Callback', @onRun);

    btnReset = uicontrol('Parent', fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [panelX, 0.05, panelW, 0.06], 'String', '重置参数', ...
        'FontSize', 11, 'Callback', @onResetParams);

    txtState = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [panelX, 0.00, panelW, 0.03], ...
        'String', '状态: 就绪 (调整参数后点击运行)', ...
        'HorizontalAlignment', 'left', 'BackgroundColor', get(fig,'Color'), ...
        'ForegroundColor', [0 0.45 0], 'FontSize', 10);

    txtHint = uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [panelX, 0.03, panelW, 0.02], ...
        'String', '命令行会持续打印实时进度', ...
        'HorizontalAlignment', 'left', 'BackgroundColor', get(fig,'Color'), ...
        'ForegroundColor', [0.2 0.2 0.2], 'FontSize', 9);

    % initial preview
    refreshPreview(true);
    fprintf('[INFO] GUI ready. 调整右侧滑条可实时更新红色抛物线，再点击 Run。\n');
    fprintf('========================= READY =========================\n\n');

    %% ---------------- nested callbacks ----------------
    function onParamChange(name, val)
        if running
            return;
        end
        params.(name) = val;
        refreshPreview(false);
    end

    function onResetParams(~, ~)
        if running
            return;
        end
        params.centerX = 1.00;
        params.yConst = 0.00;
        params.zTop = 0.20;
        params.halfSpan = 0.20;
        params.depth = 0.50;
        params.pitchBiasDeg = -12.0;
        params.assemblyOffsetDeg = 0.0;
        params.framePause = 0.03;
        params.reachRadius = estimateReachRadius(robot, baseLink, endEffector, 0.20);
        params.flipToolZ = true;

        set(sCenterX.slider, 'Value', params.centerX); set(sCenterX.valueText, 'String', sprintf('%.3f', params.centerX));
        set(sYConst.slider, 'Value', params.yConst); set(sYConst.valueText, 'String', sprintf('%.3f', params.yConst));
        set(sZTop.slider, 'Value', params.zTop); set(sZTop.valueText, 'String', sprintf('%.3f', params.zTop));
        set(sHalfSpan.slider, 'Value', params.halfSpan); set(sHalfSpan.valueText, 'String', sprintf('%.3f', params.halfSpan));
        set(sDepth.slider, 'Value', params.depth); set(sDepth.valueText, 'String', sprintf('%.3f', params.depth));
        set(sPitchBias.slider, 'Value', params.pitchBiasDeg); set(sPitchBias.valueText, 'String', sprintf('%.3f', params.pitchBiasDeg));
        set(sAssemblyOffset.slider, 'Value', params.assemblyOffsetDeg); set(sAssemblyOffset.valueText, 'String', sprintf('%.3f', params.assemblyOffsetDeg));
        set(sSpeed.slider, 'Value', params.framePause); set(sSpeed.valueText, 'String', sprintf('%.3f', params.framePause));
        set(sReach.slider, 'Value', params.reachRadius); set(sReach.valueText, 'String', sprintf('%.3f', params.reachRadius));
        set(cbFlipZ, 'Value', params.flipToolZ);

        fprintf('[GUI] 参数已重置为默认值。\n');
        refreshPreview(true);
    end

    function onFlipZChanged(src, ~)
        if running
            return;
        end
        params.flipToolZ = logical(src.Value);
        fprintf('[GUI] flipToolZ=%d\n', params.flipToolZ);
        refreshPreview(false);
    end

    function onRun(~, ~)
        if running
            fprintf('[INFO] Run ignored: already running.\n');
            return;
        end
        running = true;
        btnRun.Enable = 'off';
        txtState.String = '状态: 运行中...';
        txtState.ForegroundColor = [0 0.2 0.8];

        cleanupObj = onCleanup(@() releaseRunLock()); %#ok<NASGU>

        fprintf('\n========================= RUN =========================\n');
        thetaCompDeg = params.pitchBiasDeg + params.assemblyOffsetDeg;
        fprintf('[RUN] params: centerX=%.3f, y=%.3f, zTop=%.3f, halfSpan=%.3f, depth=%.3f, pitchBias=%.2f, assemblyOffset=%.2f, thetaComp=%.2f, zFlip=%d\n', ...
            params.centerX, params.yConst, params.zTop, params.halfSpan, params.depth, ...
            params.pitchBiasDeg, params.assemblyOffsetDeg, thetaCompDeg, params.flipToolZ);

        % recompute latest preview geometry
        refreshPreview(true);

        T_w_b = getTransform(robot, qCurrent, baseLink, baseFrame);
        R_w_b = T_w_b(1:3,1:3);
        thetaCompRad = deg2rad(thetaCompDeg);
        R_comp = axisAngleRotm([0;1;0], thetaCompRad);

        nJ = numel(qCurrent);
        qTraj = zeros(nPts, nJ);
        posErr = zeros(nPts,1);
        oriErrDeg = zeros(nPts,1);
        yAlignErrDeg = zeros(nPts,1);
        statusArr = strings(nPts,1);

        qPrev = clampToLimits(qCurrent, jointLimits);
        qHomeLocal = clampToLimits(homeConfiguration(robot), jointLimits);

        fprintf('[RUN] IK solving starts... total points=%d\n', nPts);
        for i = 1:nPts
            x_b = pathBase(i,1);
            k = 2*coeff.a*x_b + coeff.b;
            tanAngleDeg = rad2deg(atan(k));

            % Orientation in base-link frame:
            % y-axis strictly follows parabola tangent in X-Z plane.
            yAxis_b = [1; 0; k];
            yAxis_b = yAxis_b / max(norm(yAxis_b), 1e-12);

            % x-axis uses parabola-plane normal (base +Y) as reference.
            xRef_b = [0; 1; 0];
            zAxis_b = cross(xRef_b, yAxis_b);
            zAxis_b = zAxis_b / max(norm(zAxis_b), 1e-12);
            if params.flipToolZ
                zAxis_b = -zAxis_b;
            end
            xAxis_b = cross(yAxis_b, zAxis_b);
            xAxis_b = xAxis_b / max(norm(xAxis_b), 1e-12);

            % 统一Y轴补偿: 攻角偏置 + 装配偏角。
            R_b_nom = [xAxis_b, yAxis_b, zAxis_b];
            R_b = R_b_nom * R_comp;
            R_tar = R_w_b * R_b;
            yTarWorld = R_tar(:,2);

            T_tar = eye(4);
            T_tar(1:3,1:3) = R_tar;
            T_tar(1:3,4) = pathWorld(i,:)';

            seeds = [qPrev; qCurrent; qHomeLocal];
            [qBest, infoBest, ePos, eOriDeg, eYAlignDeg] = solveOneTarget(T_tar, seeds, yTarWorld);

            qTraj(i,:) = qBest;
            qPrev = qBest;
            posErr(i) = ePos;
            oriErrDeg(i) = eOriDeg;
            yAlignErrDeg(i) = eYAlignDeg;
            statusArr(i) = string(infoBest.Status);

            if i <= 5 || mod(i,10) == 0 || i == nPts
                fprintf(['[IK %03d/%03d] x=%.3f k=%.3f tan=%.2fdeg | posErr=%.4fm oriErr=%.2fdeg ' ...
                    'yAlignErr=%.2fdeg status=%s\n'], ...
                    i, nPts, x_b, k, tanAngleDeg, ePos, eOriDeg, eYAlignDeg, infoBest.Status);
            end

            if mod(i,10) == 0
                txtState.String = sprintf('状态: IK计算中 %d/%d', i, nPts);
                drawnow limitrate;
            end
        end

        fprintf('[RUN] IK done. posErr mean=%.4f max=%.4f | oriErr mean=%.2f max=%.2f | yAlignErr mean=%.2f max=%.2f (deg)\n', ...
            mean(posErr), max(posErr), mean(oriErrDeg), max(oriErrDeg), mean(yAlignErrDeg), max(yAlignErrDeg));

        % animate
        fprintf('[RUN] Animation starts...\n');
        txtState.String = '状态: 动画播放中...';
        tipTrail = nan(nPts,3);

        for i = 1:nPts
            T_now = getTransform(robot, qTraj(i,:), endEffector, baseFrame);
            tipTrail(i,:) = tform2trvec(T_now);

            renderScene(qTraj(i,:), tipTrail, i, sprintf('运行中 %d/%d', i, nPts));

            if i <= 5 || mod(i,10) == 0 || i == nPts
                fprintf('[ANIM %03d/%03d] tip=[%.3f %.3f %.3f]\n', i, nPts, tipTrail(i,1), tipTrail(i,2), tipTrail(i,3));
            end

            if mod(i,10) == 0
                txtState.String = sprintf('状态: 动画播放中 %d/%d', i, nPts);
            end
            drawnow;
            pause(params.framePause);
        end

        qCurrent = qTraj(end,:);
        finalErr = norm(tipTrail(end,:) - keyWorld.finish);
        fprintf('[RUN] Finished. final tip-to-end error = %.4fm\n', finalErr);
        fprintf('======================= RUN DONE =======================\n\n');

        txtState.String = sprintf('状态: 运行完成, 终点误差=%.4fm', finalErr);
        txtState.ForegroundColor = [0 0.45 0];
    end

    function releaseRunLock()
        running = false;
        if ishandle(btnRun)
            btnRun.Enable = 'on';
        end
    end

    %% ---------------- core helpers ----------------
    function refreshPreview(printInfo)
        [pathBase, coeff, keyBase] = buildParabolaInBase(params, nPts);
        T_w_b = getTransform(robot, qCurrent, baseLink, baseFrame);
        [pathWorld, keyWorld] = transformBasePathToWorld(pathBase, keyBase, T_w_b);

        renderScene(qCurrent, nan(nPts,3), 0, '预览');

        if printInfo
            fprintf('[PREVIEW] base points: start=[%.3f %.3f %.3f], vertex=[%.3f %.3f %.3f], end=[%.3f %.3f %.3f]\n', ...
                keyBase.start, keyBase.vertex, keyBase.finish);
            fprintf('[PREVIEW] parabola: z=%.5fx^2 + %.5fx + %.5f (in %s frame)\n', ...
                coeff.a, coeff.b, coeff.c, baseLink);
            fprintf('[PREVIEW] world points: start=[%.3f %.3f %.3f], vertex=[%.3f %.3f %.3f], end=[%.3f %.3f %.3f]\n', ...
                keyWorld.start, keyWorld.vertex, keyWorld.finish);
            openUpWorld = (keyWorld.vertex(3) < keyWorld.start(3)) && (keyWorld.vertex(3) < keyWorld.finish(3));
            fprintf('[PREVIEW] open_upward_in_world=%d | reachRadius=%.3f m | zFlip=%d\n', ...
                openUpWorld, params.reachRadius, params.flipToolZ);
        end
    end

    function renderScene(qShow, tipTrail, tipLastIdx, stageName)
        camNow = captureCameraState(ax);
        cla(ax); % clear previous preview/trajectory overlays to avoid duplicated parabolas
        show(robot, qShow, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);
        restoreCameraState(ax, camNow);

        hold(ax, 'on');
        if params.showReachSphere
            T_w_bNow = getTransform(robot, qShow, baseLink, baseFrame);
            c = tform2trvec(T_w_bNow);
            surf(ax, c(1) + params.reachRadius * sphereX, ...
                c(2) + params.reachRadius * sphereY, ...
                c(3) + params.reachRadius * sphereZ, ...
                'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.05, ...
                'EdgeColor', [0.2 0.5 0.9], 'EdgeAlpha', 0.10);
            plot3(ax, c(1), c(2), c(3), 'ms', 'MarkerSize', 7, 'LineWidth', 1.6);
        end
        plot3(ax, pathWorld(:,1), pathWorld(:,2), pathWorld(:,3), 'r-', 'LineWidth', 2.2); % desired
        plot3(ax, keyWorld.start(1), keyWorld.start(2), keyWorld.start(3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
        plot3(ax, keyWorld.vertex(1), keyWorld.vertex(2), keyWorld.vertex(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        plot3(ax, keyWorld.finish(1), keyWorld.finish(2), keyWorld.finish(3), 'bo', 'MarkerSize', 8, 'LineWidth', 2);

        if nargin >= 2 && ~isempty(tipTrail)
            validIdx = find(~isnan(tipTrail(:,1)));
            if ~isempty(validIdx)
                last = validIdx(end);
                if nargin >= 3 && tipLastIdx > 0
                    last = min(last, tipLastIdx);
                end
                plot3(ax, tipTrail(1:last,1), tipTrail(1:last,2), tipTrail(1:last,3), 'b-', 'LineWidth', 1.8);
            end
        end
        hold(ax, 'off');

        grid(ax, 'on'); axis(ax, 'equal');
        xlim(ax, xRange); ylim(ax, yRange); zlim(ax, zRange);
        xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
        title(ax, sprintf('倒挂UR10 | %s | TCP y轴 // 抛物线方向 | zFlip=%d', stageName, params.flipToolZ));
    end

    function [qBest, infoBest, posBest, oriBestDeg, yAlignBestDeg] = solveOneTarget(Ttar, seeds, yTarWorld)
        pTar = tform2trvec(Ttar);
        RTar = tform2rotm(Ttar);

        qBest = seeds(1,:);
        infoBest = struct('Status', 'N/A');
        posBest = inf;
        oriBestDeg = inf;
        yAlignBestDeg = inf;
        bestScore = inf;

        for s = 1:size(seeds,1)
            seed = clampToLimits(seeds(s,:), jointLimits);
            [qTry, infoTry] = ik(endEffector, Ttar, ikWeights, seed);
            qTry = clampToLimits(qTry, jointLimits);

            Ttry = getTransform(robot, qTry, endEffector, baseFrame);
            pTry = tform2trvec(Ttry);
            RTry = tform2rotm(Ttry);
            yTool = RTry(:,2);

            posErr = norm(pTry - pTar);
            oriErrDeg = rotmGeodesicDeg(RTry, RTar);
            c = dot(yTool, yTarWorld) / max(norm(yTool)*norm(yTarWorld), 1e-12);
            c = min(max(c, -1), 1);
            yAlignErrDeg = rad2deg(acos(c)); % ideal 0 deg

            score = posErr + 0.01 * oriErrDeg + 0.02 * yAlignErrDeg;
            if score < bestScore
                bestScore = score;
                qBest = qTry;
                infoBest = infoTry;
                posBest = posErr;
                oriBestDeg = oriErrDeg;
                yAlignBestDeg = yAlignErrDeg;
            end
        end
    end
end

function out = createSliderControl(parentFig, px, py, pw, rh, label, vmin, vmax, v0, cb)
    uicontrol('Parent', parentFig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [px, py, pw, rh*0.45], 'String', label, 'HorizontalAlignment', 'left', ...
        'BackgroundColor', get(parentFig,'Color'));

    out.slider = uicontrol('Parent', parentFig, 'Style', 'slider', 'Units', 'normalized', ...
        'Position', [px, py-rh*0.45, pw*0.78, rh*0.45], ...
        'Min', vmin, 'Max', vmax, 'Value', v0, ...
        'Callback', @(src,~) onChange(src));

    out.valueText = uicontrol('Parent', parentFig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [px+pw*0.80, py-rh*0.45, pw*0.20, rh*0.45], ...
        'String', sprintf('%.3f', v0), 'HorizontalAlignment', 'left', ...
        'BackgroundColor', get(parentFig,'Color'));

    function onChange(src)
        val = src.Value;
        out.valueText.String = sprintf('%.3f', val);
        cb(val);
    end
end

function [pathBase, coeff, keyBase] = buildParabolaInBase(params, nPts)
    pStart = [params.centerX - params.halfSpan, params.yConst, params.zTop];
    % For upside-down mounting: using +depth in base-Z makes world trajectory open upward.
    pVertex = [params.centerX, params.yConst, params.zTop + params.depth];
    pEnd = [params.centerX + params.halfSpan, params.yConst, params.zTop];

    X = [pStart(1); pVertex(1); pEnd(1)];
    Z = [pStart(3); pVertex(3); pEnd(3)];

    A = [X.^2, X, ones(3,1)];
    abc = A \ Z;

    coeff = struct('a', abc(1), 'b', abc(2), 'c', abc(3));

    xPath = linspace(pStart(1), pEnd(1), nPts)';
    yPath = ones(nPts,1) * params.yConst;
    zPath = coeff.a * xPath.^2 + coeff.b * xPath + coeff.c;
    pathBase = [xPath, yPath, zPath];

    keyBase = struct('start', pStart, 'vertex', pVertex, 'finish', pEnd);
end

function [pathWorld, keyWorld] = transformBasePathToWorld(pathBase, keyBase, T_w_b)
    n = size(pathBase,1);
    P = [pathBase, ones(n,1)]';
    Pw = (T_w_b * P)';
    pathWorld = Pw(:,1:3);

    k = [keyBase.start; keyBase.vertex; keyBase.finish];
    Kw = (T_w_b * [k, ones(3,1)]')';
    keyWorld = struct('start', Kw(1,1:3), 'vertex', Kw(2,1:3), 'finish', Kw(3,1:3));
end

function lim = collectJointLimits(robot)
    lim = zeros(0,2);
    for i = 1:robot.NumBodies
        j = robot.Bodies{i}.Joint;
        if strcmp(j.Type, 'fixed')
            continue;
        end
        l = j.PositionLimits;
        if ~isfinite(l(1)) || ~isfinite(l(2)) || l(1) >= l(2)
            l = [-pi, pi];
        end
        lim = [lim; l]; %#ok<AGROW>
    end
end

function q = clampToLimits(qIn, lim)
    q = qIn;
    n = min(numel(qIn), size(lim,1));
    epsLim = 1e-6;
    for i = 1:n
        lo = lim(i,1);
        hi = lim(i,2);
        if isfinite(lo)
            q(i) = max(q(i), lo + epsLim);
        end
        if isfinite(hi)
            q(i) = min(q(i), hi - epsLim);
        end
    end
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

function angDeg = rotmGeodesicDeg(Ra, Rb)
    Rrel = Ra * Rb';
    c = (trace(Rrel) - 1) / 2;
    c = min(max(c, -1), 1);
    angDeg = rad2deg(acos(c));
end

function R = axisAngleRotm(axis, angle)
    a = axis(:);
    na = norm(a);
    if na < 1e-12 || abs(angle) < 1e-12
        R = eye(3);
        return;
    end
    a = a / na;
    x = a(1); y = a(2); z = a(3);
    c = cos(angle);
    s = sin(angle);
    C = 1 - c;
    R = [ ...
        x*x*C + c,     x*y*C - z*s, x*z*C + y*s; ...
        y*x*C + z*s, y*y*C + c,     y*z*C - x*s; ...
        z*x*C - y*s, z*y*C + x*s, z*z*C + c];
end

function reachR = estimateReachRadius(robot, baseLink, endEffector, margin)
    if nargin < 4
        margin = 0.20;
    end

    try
        bodies = robot.BodyNames;
        if ~any(strcmp(bodies, endEffector)) || ~any(strcmp(bodies, baseLink))
            reachR = 1.8;
            return;
        end

        chain = {};
        cur = endEffector;
        maxHop = robot.NumBodies + 2;
        hop = 0;
        while ~strcmp(cur, baseLink) && hop < maxHop
            hop = hop + 1;
            chain{end+1} = cur; %#ok<AGROW>
            b = getBody(robot, cur);
            parent = b.Parent;
            if isempty(parent)
                break;
            end
            cur = parent;
        end

        if isempty(chain)
            reachR = 1.8;
            return;
        end

        L = 0;
        for i = 1:numel(chain)
            b = getBody(robot, chain{i});
            T = b.Joint.JointToParentTransform;
            L = L + norm(T(1:3,4));
        end

        reachR = max(0.6, L + margin);
    catch
        reachR = 1.8;
    end
end
