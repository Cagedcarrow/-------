function [robot, ik, weights] = initRobot()
    % INITROBOT 加载 UR10 铲刀模型并初始化运动学求解器
    
    % 1. 指定并验证 URDF 文件路径
    urdfFile = 'ur10_shovel.urdf';
    if ~exist(urdfFile, 'file')
        error('运动学模块错误: 未找到机器人模型文件 "%s"', urdfFile);
    end
    
    % 2. 导入机器人并设置数据格式
    robot = importrobot(urdfFile);
    robot.DataFormat = 'row'; 

    % 3. 初始化逆运动学 (IK) 求解器
    ik = inverseKinematics('RigidBodyTree', robot);
    
    % 4. 配置求解权重
    weights = [1 1 1 1 1 1]; 

    % 5. 状态反馈 (修复点：删除不存在的 robot.Name)
    % 我们可以通过打印文件路径或者直接打印 robot 对象来确认加载成功
    fprintf('--> [Kinematics] 机器人模型已成功从 "%s" 加载。\n', urdfFile);
    fprintf('--> [Kinematics] 包含 %d 个刚体，IK 求解器已准备就绪。\n', robot.NumBodies);
end