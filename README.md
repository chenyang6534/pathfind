# pathfind
Minkowski Sum + Visibility Graph + A* 完整算法流程：  Minkowski Sum（闵可夫斯基和扩展）  将矩形移动者简化为点 将所有障碍物按移动者的半径扩展 这样就把矩形碰撞问题转化为点-多边形碰撞问题 Visibility Graph（可视图构建）  提取扩展后障碍物的角点作为图节点 添加起点和终点作为额外节点 计算节点间的可见性（用空间哈希优化） A* Search（A*搜索）  在可视图上执行A*算法找最短路径 使用欧几里得距离作为启发函数 Path Smoothing（路径平滑）  移除路径中的冗余节点 优化最终路径
