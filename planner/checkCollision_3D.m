function isColliding = checkCollision_3D(q_near, q_new, obs_matrix)
    % 此函数被 Algo_DP_RRT_3D 内部调用
    
    % 获取主程序传入的机器人句柄 (通过 persistent 变量或参数传递，此处演示逻辑)
    % 在集成时，建议将 robot 存入全局或作为参数传递
    fig = gcf;
    h = guidata(fig);
    
    % 1. 基础环境碰撞检查 (使用你原本的距离计算逻辑)
    % 如果距离障碍物太近，直接返回碰撞
    distToObs = compute_3D_dmin_hetero(q_new, obs_matrix);
    if distToObs < 0.02
        isColliding = true; return;
    end

    % 2. 机械臂可达性与自碰撞检查 (核心集成点)
    % 将 3D 点转化为关节角
    targetTform = trvec2tform(q_new); % 默认姿态
    [config, success] = solveIK(h.ik, 'shovel_tip', targetTform, h.weights, h.currentConfig);
    
    if ~success
        isColliding = true; % 无法到达的点视为碰撞
        return;
    end
    
    % 检查该姿态下，铲子是否撞到机械臂本体
    isColliding = collisionChecker(h.robot, config);
end