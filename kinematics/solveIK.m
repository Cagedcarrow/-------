function [newConfig, success] = solveIK(ik, endEffector, targetTform, weights, initialGuess)
    % SOLVEIK 执行逆运动学求解计算
    % 输入:
    %   ik           - 由 initRobot 初始化生成的 inverseKinematics 求解器对象
    %   endEffector  - 需要控制的末端坐标系名称，此处为 'shovel_tip'
    %   targetTform  - 4x4 目标齐次变换矩阵，包含目标位置和姿态
    %   weights      - 6位权重向量，定义位置与姿态的求解优先级
    %   initialGuess - 求解迭代的初始猜测值（通常为当前关节配置），用于保证运动连续性
    % 输出:
    %   newConfig    - 求解出的关节配置（1x6 行向量）
    %   success      - 布尔值，指示求解是否成功或达到可用精度

    % 1. 调用求解器
    % 该函数通过迭代最小化末端位姿误差来寻找最优关节解
    [newConfig, info] = ik(endEffector, targetTform, weights, initialGuess);

    % 2. 验证求解状态
    % 'success': 误差在预设容差范围内
    % 'best available': 虽然未完全达到容差，但已找到局部最优解，在工作空间边缘常出现
    success = strcmp(info.Status, 'success') || strcmp(info.Status, 'best available');

    % 3. 异常处理
    if ~success
        % 在学术仿真中，记录求解失败的状态有助于分析机械臂的工作空间死角
        fprintf('--> [Kinematics] 警告: IK 求解未成功。当前状态: %s\n', info.Status); 
    end
end