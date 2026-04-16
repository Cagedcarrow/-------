function verifySTLModels()
    % 验证所有STL碰撞模型是否正确加载
    
    fprintf('=== 验证STL碰撞模型 ===\n');
    
    % 加载机器人
    [robot, ~, ~] = initRobot();
    
    % 获取初始配置
    q0 = homeConfiguration(robot);
    
    % 检查每个刚体的碰撞几何体
    for i = 1:robot.NumBodies
        body = robot.Bodies{i};
        bodyName = body.Name;
        
        fprintf('\n刚体: %s\n', bodyName);
        
        if ~isempty(body.Collisions)
            collision = body.Collisions{1};
            fprintf('  碰撞几何体类型: %s\n', collision.Geometry.Type);
            
            if strcmp(collision.Geometry.Type, 'Mesh')
                fprintf('  网格文件: %s\n', collision.Geometry.Mesh.Filename);
                fprintf('  顶点数量: %d\n', size(collision.Mesh.Vertices, 1));
                fprintf('  面数量: %d\n', size(collision.Mesh.Faces, 1));
                
                % 显示包围盒
                vertices = collision.Mesh.Vertices;
                bbox_min = min(vertices);
                bbox_max = max(vertices);
                fprintf('  包围盒: [%.3f,%.3f,%.3f] to [%.3f,%.3f,%.3f]\n', ...
                       bbox_min, bbox_max);
            end
        else
            fprintf('  警告: 没有碰撞几何体\n');
        end
    end
    
    fprintf('\n=== 验证完成 ===\n');
end