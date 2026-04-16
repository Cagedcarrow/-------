function shovel_tcp_move(urdfFile, endEffector)
% TCP-frame incremental control for shovel tip (dX,dY,dZ,dYaw,dPitch,dRoll).
% Usage:
%   shovel_tcp_move
%   shovel_tcp_move('ur10_world.urdf','shovel_tip')

    fprintf('[INFO] script=%s\n', mfilename('fullpath'));
    fprintf('[INFO] version=2026-04-16-ee-slider-v5-tcp-frame\n');

    if nargin < 1 || isempty(urdfFile), urdfFile = 'ur10_world.urdf'; end
    if nargin < 2 || isempty(endEffector), endEffector = 'shovel_tip'; end

    robot = importrobot(urdfFile);
    robot.DataFormat = 'row';
    robot.Gravity = [0 0 -9.81];
    if ~any(strcmp(robot.BodyNames, endEffector))
        error('[ERROR] endEffector "%s" not found.', endEffector);
    end

    ik = inverseKinematics('RigidBodyTree', robot);
    ikWeights = [1 1 1 0.08 0.08 0.08];

    q = homeConfiguration(robot);
    refT = getTransform(robot, q, endEffector);

    f = figure('Name', 'Shovel TCP Move', 'NumberTitle', 'off', ...
        'Position', [80 80 1360 780], 'Color', [0.97 0.97 0.97]);

    ax = axes('Parent', f, 'Units', 'normalized', 'Position', [0.04 0.08 0.62 0.86]);
    view(ax, 135, 25); grid(ax, 'on'); axis(ax, 'equal');
    xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
    title(ax, sprintf('TCP Frame Increment | URDF: %s | EE: %s', urdfFile, endEffector), 'Interpreter', 'none');
    xlim(ax, [-2.5, 2.5]); ylim(ax, [-2.5, 2.5]); zlim(ax, [-2.5, 2.5]);

    panelX = 0.70; panelW = 0.28; topY = 0.89; rowH = 0.11;
    names = {'dX_tcp (m)','dY_tcp (m)','dZ_tcp (m)','dYaw_tcp (deg)','dPitch_tcp (deg)','dRoll_tcp (deg)'};
    mins = [-0.8, -0.8, -0.8, -120, -120, -120];
    maxs = [ 0.8,  0.8,  0.8,  120,  120,  120];
    vals = zeros(1,6);

    sliders = gobjects(1,6);
    valueTexts = gobjects(1,6);
    txtStatus = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [panelX, 0.20, panelW, 0.035], 'String', '状态: TCP增量模式就绪', ...
        'HorizontalAlignment', 'left', 'BackgroundColor', f.Color, 'ForegroundColor', [0 0.45 0]);

    for i = 1:6
        y = topY - (i-1) * rowH;
        uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [panelX, y, panelW, 0.035], 'String', names{i}, ...
            'HorizontalAlignment', 'left', 'BackgroundColor', f.Color);
        sliders(i) = uicontrol('Parent', f, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [panelX, y-0.045, panelW*0.78, 0.035], ...
            'Min', mins(i), 'Max', maxs(i), 'Value', vals(i), ...
            'Callback', @(src,~) onPoseSliderChanged(i, src.Value));
        valueTexts(i) = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [panelX+panelW*0.80, y-0.045, panelW*0.20, 0.035], ...
            'String', '0.000', 'HorizontalAlignment', 'left', 'BackgroundColor', f.Color);
    end

    uicontrol('Parent', f, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [panelX, 0.12, 0.12, 0.05], 'String', 'Reset Home', ...
        'Callback', @(~,~) onResetHome());
    uicontrol('Parent', f, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [panelX+0.14, 0.12, 0.12, 0.05], 'String', '设为TCP参考', ...
        'Callback', @(~,~) onSetReference());

    renderRobot();

    function onPoseSliderChanged(idx, val)
        vals(idx) = min(max(val, mins(idx)), maxs(idx));
        valueTexts(idx).String = sprintf('%.3f', vals(idx));

        dPosTCP = vals(1:3)';
        dEulTCP = deg2rad(vals(4:6));

        Rref = refT(1:3,1:3);
        pref = refT(1:3,4);
        Roff = eul2rotm(dEulTCP, 'ZYX');

        targetPos = pref + Rref * dPosTCP;
        targetRot = Rref * Roff;
        targetT = eye(4);
        targetT(1:3,1:3) = targetRot;
        targetT(1:3,4) = targetPos;

        [qTry, solInfo] = ik(endEffector, targetT, ikWeights, q);
        eeTry = getTransform(robot, qTry, endEffector);
        posErr = norm(tform2trvec(eeTry) - targetPos');

        if posErr > 0.15
            txtStatus.String = sprintf('状态: IK误差过大(拒绝) %.3fm', posErr);
            txtStatus.ForegroundColor = [0.8 0.25 0.0];
            fprintf('[DEBUG] reject posErr=%.4f, idx=%d, status=%s\n', posErr, idx, solInfo.Status);
            return;
        end

        q = qTry;
        if posErr > 0.05
            txtStatus.String = sprintf('状态: 可执行(有误差) %.3fm | %s', posErr, solInfo.Status);
            txtStatus.ForegroundColor = [0.8 0.45 0.0];
        else
            txtStatus.String = sprintf('状态: IK成功 %.3fm | %s', posErr, solInfo.Status);
            txtStatus.ForegroundColor = [0 0.45 0];
        end
        renderRobot();
    end

    function onSetReference()
        refT = getTransform(robot, q, endEffector);
        resetOffsets();
        p = tform2trvec(refT);
        fprintf('[INFO] TCP reference updated. p=[%.4f %.4f %.4f]\n', p);
        txtStatus.String = '状态: 已更新TCP参考坐标系';
        txtStatus.ForegroundColor = [0 0.45 0];
    end

    function onResetHome()
        q = homeConfiguration(robot);
        refT = getTransform(robot, q, endEffector);
        resetOffsets();
        renderRobot();
        txtStatus.String = '状态: 已重置 Home + TCP参考';
        txtStatus.ForegroundColor = [0 0.45 0];
    end

    function resetOffsets()
        vals(:) = 0;
        for k = 1:6
            sliders(k).Value = 0;
            valueTexts(k).String = '0.000';
        end
    end

    function renderRobot()
        camNow = captureCameraState(ax);
        show(robot, q, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);
        restoreCameraState(ax, camNow);
        grid(ax, 'on');
        drawnow limitrate;
    end
end

function s = captureCameraState(ax)
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
