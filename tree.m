function tree(dirname, prefix, isLast)
    % TREE 显示目录结构，类似Linux的tree命令
    % 用法：
    %   tree                    % 显示当前目录结构
    %   tree('dirname')         % 显示指定目录结构
    %   tree('dirname', prefix, isLast)  % 递归调用参数
    
    % 处理输入参数
    if nargin == 0
        dirname = pwd;          % 默认当前目录
        prefix = '';
        isLast = true;
    elseif nargin == 1
        prefix = '';
        isLast = true;
    end
    
    % 获取目录信息
    files = dir(dirname);
    
    % 过滤掉 '.' 和 '..'
    files = files(~ismember({files.name}, {'.', '..'}));
    
    % 分离文件夹和文件
    isDir = [files.isdir];
    dirs = files(isDir);
    regularFiles = files(~isDir);
    
    % 组合所有条目（先文件夹后文件）
    allItems = [dirs; regularFiles];
    
    % 遍历并显示
    for i = 1:length(allItems)
        item = allItems(i);
        isLastItem = (i == length(allItems));
        
        % 确定连接符号
        if isLastItem
            connector = '└── ';
            childPrefix = [prefix, '    '];
        else
            connector = '├── ';
            childPrefix = [prefix, '│   '];
        end
        
        % 显示当前条目
        fprintf('%s%s%s', prefix, connector, item.name);
        
        % 如果是文件夹，显示 '/' 后缀并递归
        if item.isdir
            fprintf('/\n');
            fullPath = fullfile(dirname, item.name);
            tree(fullPath, childPrefix, isLastItem);
        else
            fprintf('\n');
        end
    end
end