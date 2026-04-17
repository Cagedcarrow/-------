function ur10_joint_slider_control(urdfFile)
% UR10 6-DOF slider control demo for URDF models.
% Usage:
%   ur10_joint_slider_control
%   ur10_joint_slider_control('ur10_world.urdf')

    fprintf('[INFO] script=%s\n', mfilename('fullpath'));
    fprintf('[INFO] version=2026-04-16-joint-slider-v3\n');

    if nargin < 1 || isempty(urdfFile)
        urdfFile = 'ur10_world.urdf';
    end

    fprintf('[INFO] Loading URDF: %s\n', urdfFile);
    if ~exist(urdfFile, 'file')
        error('[ERROR] URDF not found: %s', urdfFile);
    end

    try
        robot = importrobot(urdfFile);
        robot.DataFormat = 'row';
        robot.Gravity = [0 0 -9.81];
    catch ME
        fprintf('[DEBUG] importrobot failed. message=%s\n', ME.message);
        rethrow(ME);
    end

    robot = applyUpsideDownBase(robot);

    [jointNames, jointLimits] = getRevoluteJointInfo(robot);
    n = numel(jointNames);

    fprintf('[INFO] Detected revolute joints: %d\n', n);
    for i = 1:n
        fprintf('  %d) %s   [%.3f, %.3f] rad\n', i, jointNames{i}, jointLimits(i,1), jointLimits(i,2));
    end

    if n < 6
        error('[ERROR] Expected at least 6 revolute joints, but got %d.', n);
    end

    jointNames = jointNames(1:6);
    jointLimits = jointLimits(1:6, :);
    q = zeros(1, 6);

    f = figure('Name', 'UR10 Joint Slider Control', 'NumberTitle', 'off', ...
        'Position', [80 80 1280 760], 'Color', [0.97 0.97 0.97]);

    ax = axes('Parent', f, 'Units', 'normalized', 'Position', [0.05 0.08 0.62 0.86]);
    fprintf('[INFO] axes class=%s\n', class(ax));
    view(ax, 135, 25);
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
    title(ax, sprintf('URDF: %s', urdfFile), 'Interpreter', 'none');
    setFixedWorkspace(ax);

    sliders = gobjects(1,6);
    valueTexts = gobjects(1,6);

    panelX = 0.72;
    panelW = 0.25;
    topY = 0.88;
    rowH = 0.11;

    for i = 1:6
        y = topY - (i-1) * rowH;

        uicontrol('Parent', f, 'Style', 'text', ...
            'Units', 'normalized', ...
            'Position', [panelX, y, panelW, 0.035], ...
            'String', sprintf('J%d: %s', i, jointNames{i}), ...
            'HorizontalAlignment', 'left', 'BackgroundColor', f.Color);

        sliders(i) = uicontrol('Parent', f, 'Style', 'slider', ...
            'Units', 'normalized', ...
            'Position', [panelX, y-0.045, panelW*0.78, 0.035], ...
            'Min', jointLimits(i,1), 'Max', jointLimits(i,2), 'Value', q(i), ...
            'Callback', @(src,~) onSliderChanged(i, src.Value));

        valueTexts(i) = uicontrol('Parent', f, 'Style', 'text', ...
            'Units', 'normalized', ...
            'Position', [panelX+panelW*0.80, y-0.045, panelW*0.20, 0.035], ...
            'String', sprintf('%.3f', q(i)), ...
            'HorizontalAlignment', 'left', 'BackgroundColor', f.Color);
    end

    uicontrol('Parent', f, 'Style', 'pushbutton', ...
        'Units', 'normalized', 'Position', [panelX, 0.12, 0.11, 0.05], ...
        'String', 'Reset Home', 'Callback', @(~,~) onReset());

    uicontrol('Parent', f, 'Style', 'pushbutton', ...
        'Units', 'normalized', 'Position', [panelX+0.13, 0.12, 0.11, 0.05], ...
        'String', 'Print q', 'Callback', @(~,~) fprintf('[q] %s\n', mat2str(q, 5)));

    camState = [];
    renderRobot();

    % 初始化相机状态；后续每次重绘前都会重新抓取当前状态
    camState = captureCameraState(ax);

    function onSliderChanged(idx, val)
        q(idx) = clamp(val, jointLimits(idx,1), jointLimits(idx,2));
        valueTexts(idx).String = sprintf('%.3f', q(idx));
        renderRobot();
    end

    function onReset()
        q(:) = 0;
        for k = 1:6
            sliders(k).Value = q(k);
            valueTexts(k).String = sprintf('%.3f', q(k));
        end
        fprintf('[INFO] Reset to q=zeros(1,6).\n');
        renderRobot();
    end

    function renderRobot()
        try
            % 先抓当前相机状态，避免 show() 清空后回到默认视角
            if ~isempty(camState)
                camState = captureCameraState(ax);
            end

            show(robot, q, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);

            if ~isempty(camState)
                restoreCameraState(ax, camState);
            else
                view(ax, 135, 25);
            end

            grid(ax, 'on');
            drawnow limitrate;
        catch ME
            fprintf('[DEBUG] show() failed. q=%s\n', mat2str(q, 6));
            fprintf('[DEBUG] error=%s\n', ME.message);
        end
    end
end

function [jointNames, jointLimits] = getRevoluteJointInfo(robot)
    jointNames = {};
    jointLimits = [];

    for i = 1:robot.NumBodies
        j = robot.Bodies{i}.Joint;
        if strcmp(j.Type, 'revolute')
            jointNames{end+1,1} = j.Name; %#ok<AGROW>
            lim = j.PositionLimits;

            if ~isfinite(lim(1)) || ~isfinite(lim(2)) || lim(1) >= lim(2)
                lim = [-pi, pi];
            end
            if abs(lim(1) - lim(2)) < 1e-6
                lim = [-pi, pi];
            end

            jointLimits = [jointLimits; lim]; %#ok<AGROW>
        end
    end
end

function robot = applyUpsideDownBase(robot)
    rootBody = robot.Bodies{1};
    t0 = rootBody.Joint.JointToParentTransform;
    tFlip = eul2tform([0, 0, pi], 'ZYX'); % roll=pi，倒挂
    tNew = t0 * tFlip;
    setFixedTransform(rootBody.Joint, tNew);
    fprintf('[INFO] Upside-down base enabled. root=%s, joint=%s\n', rootBody.Name, rootBody.Joint.Name);
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function setFixedWorkspace(ax)
    xlim(ax, [-2.5, 2.5]);
    ylim(ax, [-2.5, 2.5]);
    zlim(ax, [-2.5, 2.5]);
    axis(ax, 'equal');
    ax.XLimMode = 'manual';
    ax.YLimMode = 'manual';
    ax.ZLimMode = 'manual';
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
