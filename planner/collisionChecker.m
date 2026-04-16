function [isColliding, details] = collisionChecker(robot, config, verbose)
    % COLLISIONCHECKER 检查机器人当前配置下是否存在自碰撞
    % 输入:
    %   robot  - rigidBodyTree 对象
    %   config - 待检测的关节配置 (1x6 行向量)
    %   verbose - 是否打印告警 (可选，默认 true)
    % 输出:
    %   isColliding - 布尔值，1 表示发生碰撞，0 表示安全
    if nargin < 3
        verbose = true;
    end

    % 1. 执行自碰撞检测
    % 该函数会读取 URDF 中的 <collision> 几何体定义进行干涉计算
    [isColliding, details] = checkCollision(robot, config, ...
        'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');

    % 2. 如果发生碰撞，在命令行给予提示
    if isColliding && verbose
        fprintf('--> [Planner] 警告：在当前路径点检测到自碰撞！\n');
    end
end
