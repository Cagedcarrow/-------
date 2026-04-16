function [startPos, endPos] = refreshDisplay(handles)
    % 检查 handles 是否包含必要字段，防止初始化期间的误触发
    if ~isfield(handles, 'startSliders') || ~isfield(handles, 'endSliders')
        startPos = []; endPos = [];
        return;
    end

    % 提取起始点坐标
    sX = handles.startSliders{1}.Value;
    sY = handles.startSliders{2}.Value;
    sZ = handles.startSliders{3}.Value;
    startPos = [sX, sY, sZ];

    % 提取终点坐标
    eX = handles.endSliders{1}.Value;
    eY = handles.endSliders{2}.Value;
    eZ = handles.endSliders{3}.Value;
    endPos = [eX, eY, eZ];

    % 更新视觉标记
    set(handles.hStart, 'XData', sX, 'YData', sY, 'ZData', sZ);
    set(handles.hEnd, 'XData', eX, 'YData', eY, 'ZData', eZ);
    
    drawnow limitrate;
end