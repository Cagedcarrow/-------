function pathPoints = linearInterpolator(startPos, endPos, numSteps)
    % LINEARINTERPOLATOR 生成起始点与终点之间的笛卡尔空间直线路径
    % 输入:
    %   startPos - 起始点坐标 [x, y, z]
    %   endPos   - 终点坐标 [x, y, z]
    %   numSteps - 插值步数（步数越多，运动越平滑）
    % 输出:
    %   pathPoints - numSteps x 3 的矩阵，每一行为一个路径点的 XYZ 坐标

    % 1. 输入验证
    % 确保输入为行向量，便于矩阵计算
    startPos = reshape(startPos, 1, 3);
    endPos = reshape(endPos, 1, 3);

    % 2. 执行线性插值
    % 在每个维度（X, Y, Z）上生成等间距的坐标序列
    % 算法公式: P(t) = P_start + t * (P_end - P_start), t ∈ [0, 1]
    px = linspace(startPos(1), endPos(1), numSteps);
    py = linspace(startPos(2), endPos(2), numSteps);
    pz = linspace(startPos(3), endPos(3), numSteps);

    % 3. 合并为路径矩阵
    % 每一行代表机械臂末端 shovel_tip 在某一时刻应达到的空间位置
    pathPoints = [px', py', pz'];

    % 4. 状态反馈（可选）
    % fprintf('--> [Planner] 直线路径生成完成，共 %d 个离散点。\n', numSteps);
end