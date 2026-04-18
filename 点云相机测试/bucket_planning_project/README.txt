Bucket Planning Project (独立总工程)

目录说明:
- main: 三个主程序和启动脚本
- modules: perception / planning / control / simulation / common
- data: 运行生成的 pcd 和 mat 数据
- meshes + textures + ur10_world.urdf: 机器人模型资源
- legacy: 旧脚本备份

推荐运行顺序 (在本目录下):
1) run('main/run_app1.m')
2) run('main/run_app2.m')
3) run('main/run_app4.m')   % 可选: TCP局部坐标系手动移动验算
4) run('main/run_app3.m')
5) run('main/run_app5.m')   % 新: 读取PCD构建STL并用CUDA加速RRT*规划

说明:
- 主程序内部已添加路径自举逻辑，会自动加载 modules/common。
- 如需完全重置数据，可手动清空 data 目录后重新执行步骤 1-3。
