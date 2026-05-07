# Path Finding Algorithms 学习项目

这个目录用同一个二维网格地图对比四种经典寻路算法：

- Depth-First Search, DFS，深度优先搜索
- Breadth-First Search, BFS，广度优先搜索
- Dijkstra，迪杰斯特拉算法
- A*，A star 算法

实验目标是直观看到它们在“是否保证最短路径、搜索范围、运行时间、适用场景”上的差异。

## 问题设定

地图是一个二维网格：

```text
S = 起点
G = 终点
# = 障碍物
. = 可通行格子
```

每一步只能上下左右移动，不能斜着走。当前脚本使用等权地图，也就是每走一步代价都为 `1`。

## 四种算法核心区别

| 算法 | 是否使用启发式 | 是否保证最短路径 | 主要特点 | 典型适用场景 |
| --- | --- | --- | --- | --- |
| DFS | 否 | 否 | 一条路走到底，内存占用小，但容易绕远 | 迷宫遍历、连通性检查 |
| BFS | 否 | 是，等权图中保证 | 一层一层扩展，能找到步数最少路径 | 等权网格、最短步数 |
| Dijkstra | 否 | 是，非负权图中保证 | 按当前总代价最小扩展，比 BFS 更通用 | 带不同道路成本的地图 |
| A* | 是 | 是，启发式可采纳时保证 | 用启发式朝目标搜索，通常访问节点更少 | 游戏寻路、机器人路径规划 |

## 文件说明

```text
03_Path Finding Algorithms/
├── README.md
├── pathfinding_comparison.py
├── requirements.txt
└── outputs/
    └── pathfinding_comparison.png
```

## 运行方式

在仓库根目录运行：

```powershell
python "03_Path Finding Algorithms/pathfinding_comparison.py"
```

脚本会输出每种算法的：

- 是否找到路径
- 路径步数
- 访问节点数量
- 运行时间

并生成对比图：

```text
03_Path Finding Algorithms/outputs/pathfinding_comparison.png
```

如果缺少依赖，先安装：

```powershell
python -m pip install -r "03_Path Finding Algorithms/requirements.txt"
```

## 图中看什么

- 蓝色路线：算法最终找到的路径。
- 灰色区域：障碍物。
- 浅蓝色区域：算法搜索过的节点。
- 绿色点：起点。
- 红色点：终点。
- 底部柱状图：路径长度、访问节点数和运行时间对比。

## 一句话理解

- DFS：能找路，但不关心是不是最短。
- BFS：在等权地图中能找最短路，但搜索范围可能大。
- Dijkstra：能处理非负权重，比 BFS 更通用。
- A*：在知道目标方向时，通常用更少搜索找到最短路。
