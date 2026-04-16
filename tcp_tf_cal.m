function calculate_shovel_rotation_matrix()
    % 1. 加载包含铲子和世界坐标系的 URDF 模型
    urdfFile = 'ur10_world.urdf';
    fprintf('[INFO] 正在加载 URDF: %s\n', urdfFile);
    robot = importrobot(urdfFile);
    robot.DataFormat = 'row'; % 设置数据格式为行向量，与你的系统保持一致
    
    % 2. 定义当前机械臂的关节角 q (可根据需要修改为任意有效位形)
    % 这里的顺序对应: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    q = [0.5000, -0.7854, 1.5708, -0.7854, 1.5708, 0]; 
    fprintf('[INFO] 当前关节角 q = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', q);
    
    % 3. 定义参考坐标系和目标坐标系
    % 在你重构的模型中，全局基座是 'world_base'，真正的末端作业点是 'shovel_tip'
    baseFrame = 'world_base'; 
    tcpFrame = 'shovel_tip';  
    
    % 4. 调用正运动学函数，获取末端在基坐标系下的齐次变换矩阵 T (4x4)
    % 该函数底层自动计算了你论文中的 T_tcp = T_6^0 * T_fn
    T_tcp = getTransform(robot, q, tcpFrame, baseFrame);
    
    % 5. 提取旋转矩阵 R (3x3) 和 平移向量 P (3x1)
    % 齐次矩阵的前三行前三列即为旋转矩阵
    R_tcp = T_tcp(1:3, 1:3); 
    % 齐次矩阵的前三行第四列即为三维位置坐标
    P_tcp = T_tcp(1:3, 4);   
    
    % 6. 将旋转矩阵转换为人类直观理解的欧拉角 (Roll, Pitch, Yaw)
    % 'ZYX' 顺序对应: 绕Z轴旋转(Yaw), 绕Y轴旋转(Pitch), 绕X轴旋转(Roll)
    eul_ZYX = rotm2eul(R_tcp, 'ZYX'); 
    yaw   = eul_ZYX(1);
    pitch = eul_ZYX(2);
    roll  = eul_ZYX(3);
    
    % ================= 打印输出结果 =================
    fprintf('\n=== 正运动学计算结果 ===\n');
    disp('1. 齐次变换矩阵 T_tcp (4x4):');
    disp(T_tcp);
    
    disp('2. 末端旋转矩阵 R_tcp (3x3):');
    disp(R_tcp);
    
    fprintf('3. 末端空间位置 P_tcp (X, Y, Z) 米:\n');
    fprintf('   [%.4f, %.4f, %.4f]\n\n', P_tcp(1), P_tcp(2), P_tcp(3));
    
    fprintf('4. 铲子姿态欧拉角 (弧度):\n');
    fprintf('   Roll (横滚)  = %.4f rad\n', roll);
    fprintf('   Pitch(俯仰)  = %.4f rad (切入角)\n', pitch);
    fprintf('   Yaw  (偏航)  = %.4f rad\n', yaw);
    
    fprintf('   铲子姿态欧拉角 (角度):\n');
    fprintf('   Roll  = %.2f°\n', rad2deg(roll));
    fprintf('   Pitch = %.2f°\n', rad2deg(pitch));
    fprintf('   Yaw   = %.2f°\n', rad2deg(yaw));
end