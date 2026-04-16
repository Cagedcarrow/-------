function handles = MoveShovelTip(robot, runCallback)
    % 1. 创建窗口
    handles.fig = figure('Name', 'UR10 路径规划系统', 'Position', [100 100 1200 700], 'Color', 'w');

    % 2. 创建坐标轴
    handles.ax = axes('Parent', handles.fig, 'Position', [0.35 0.12 0.6 0.8]);
    hold(handles.ax, 'on'); grid(handles.ax, 'on'); axis(handles.ax, 'equal');
    view(handles.ax, 135, 30);
    xlim(handles.ax, [-1.5, 1.5]); ylim(handles.ax, [-1.5, 1.5]); zlim(handles.ax, [0, 2.2]);
    show(robot, homeConfiguration(robot), 'Parent', handles.ax, 'PreservePlot', false);

    % 3. 初始标记
    initStart = [0.6, -0.4, 1.0];
    initEnd   = [0.6, 0.4, 1.0];
    handles.hStart = plot3(handles.ax, initStart(1), initStart(2), initStart(3), 'go', 'MarkerSize', 12, 'LineWidth', 3);
    handles.hEnd = plot3(handles.ax, initEnd(1), initEnd(2), initEnd(3), 'ro', 'MarkerSize', 12, 'LineWidth', 3);

    % --- 关键修复：先创建面板和滑块，暂不绑定回调 ---
    pnlS = uipanel('Parent', handles.fig, 'Title', '起始点 (Green)', 'Position', [0.02 0.55 0.3 0.4]);
    handles.startSliders = createXYZSliders(pnlS, initStart, []); % 传入空回调

    pnlE = uipanel('Parent', handles.fig, 'Title', '目标点 (Red)', 'Position', [0.02 0.15 0.3 0.4]);
    handles.endSliders = createXYZSliders(pnlE, initEnd, []); % 传入空回调

    % 4. 创建"开始运行"按钮
    handles.btnRun = uicontrol('Parent', handles.fig, 'Style', 'pushbutton', ...
        'String', '▶ 开始规划运行', 'Units', 'normalized', 'Position', [0.02 0.03 0.3 0.08], ...
        'Callback', runCallback);

    % --- 关键修复：现在所有字段都存在了，统一绑定回调并保存数据 ---
    % 使用匿名函数获取最新的 guidata
    refreshFunc = @(src, evt) refreshDisplay(guidata(handles.fig));
    
    for i = 1:3
        set(handles.startSliders{i}, 'Callback', refreshFunc);
        set(handles.endSliders{i}, 'Callback', refreshFunc);
    end
    
    % 将 handles 存入 Figure
    guidata(handles.fig, handles);
end

function sH = createXYZSliders(parent, initV, callback)
    names = {'X','Y','Z'};
    sH = cell(1,3);
    for i=1:3
        y = 1 - i*0.28;
        uicontrol('Style','text','String',names{i},'Units','normalized','Position',[0.05 y+0.1 0.2 0.1],'Parent',parent);
        sH{i} = uicontrol('Style','slider','Units','normalized','Position',[0.05 y 0.9 0.1],...
            'Min',-1.2,'Max',1.2,'Value',initV(i),'Parent',parent,'Callback',callback);
    end
end