% 读取STL文件
TR = stlread('避障场景.STL');

% 交换Y和Z坐标（将[Y, Z]改为[Z, Y]）
points_swapped = TR.Points;
points_swapped(:, [2, 3]) = points_swapped(:, [3, 2]);  % 交换第2列和第3列

% 计算模型的长宽高（在交换后的坐标系中）
% X轴方向长度
x_min = min(points_swapped(:,1));
x_max = max(points_swapped(:,1));
x_length = x_max - x_min;

% Y轴方向长度（原Z轴）
y_min = min(points_swapped(:,2));
y_max = max(points_swapped(:,2));
y_length = y_max - y_min;

% Z轴方向长度（原Y轴）
z_min = min(points_swapped(:,3));
z_max = max(points_swapped(:,3));
z_length = z_max - z_min;

% 打印模型尺寸信息
fprintf('=== 模型尺寸信息 ===\n');
fprintf('坐标系说明：X轴不变，Y轴为原Z轴，Z轴为原Y轴\n');
fprintf('X方向长度: %.2f 单位\n', x_length);
fprintf('Y方向长度: %.2f 单位\n', y_length);
fprintf('Z方向长度: %.2f 单位\n', z_length);
fprintf('总尺寸: %.2f × %.2f × %.2f 单位\n', x_length, y_length, z_length);
fprintf('注意：单位取决于STL文件的原始单位（通常是毫米或米）\n');
fprintf('===================\n\n');

% 使用交换后的坐标绘制
trisurf(TR.ConnectivityList, points_swapped(:,1), points_swapped(:,2), points_swapped(:,3), ...
    'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none');
axis equal;
view(3);
xlabel('X'); ylabel('Z'); zlabel('Y');  % 注意标签也交换了
title('避障场景 (Y/Z交换后)');
camlight;
lighting gouraud;