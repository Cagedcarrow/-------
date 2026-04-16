function baseHandles = BaseControl(updateCallback)
    % BASECONTROL 创建独立的机器人基座姿态控制窗口
    % 输入: updateCallback - 当滑块变动时，主程序执行的刷新函数句柄

    % 1. 创建独立窗口
    baseHandles.fig = figure('Name', '基座姿态控制面板', 'NumberTitle', 'off', ...
                             'MenuBar', 'none', 'Position', [1320 100 350 550], 'Color', 'w');

    % 2. 定义标签和范围
    labels = {'基座 X (m)', '基座 Y (m)', '基座 Z (m)', 'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'};
    limits = [-2 2; -2 2; 0 1; -180 180; -180 180; -180 180];
    initVals = [0 0 0 0 0 0];

    baseHandles.sliders = cell(1, 6);
    baseHandles.txtVals = cell(1, 6);

    % 3. 循环创建 6 个控制组
    for i = 1:6
        y_pos = 550 - i * 85;
        % 标签
        uicontrol('Parent', baseHandles.fig, 'Style', 'text', 'String', labels{i}, ...
            'Position', [20 y_pos+40 150 20], 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
        
        % 数值显示
        baseHandles.txtVals{i} = uicontrol('Parent', baseHandles.fig, 'Style', 'text', 'String', num2str(initVals(i)), ...
            'Position', [250 y_pos+40 80 20], 'BackgroundColor', 'w', 'HorizontalAlignment', 'right');
        
        % 滑块
        baseHandles.sliders{i} = uicontrol('Parent', baseHandles.fig, 'Style', 'slider', ...
            'Position', [20 y_pos 310 30], 'Min', limits(i,1), 'Max', limits(i,2), ...
            'Value', initVals(i), 'Callback', updateCallback);
        guidata(baseHandles.fig, baseHandles);
    end
end