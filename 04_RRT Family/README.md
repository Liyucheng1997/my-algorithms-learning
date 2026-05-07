# RRT Family 学习项目

这个目录用同一个二维连续空间地图，对比 RRT family 中常见的三种算法：

- RRT, Rapidly-exploring Random Tree
- RRT-Connect
- RRT*, RRT Star

RRT family 和 BFS、Dijkstra、A* 这类网格/图搜索不同：它们不要求提前把空间离散成网格，而是在连续空间里随机采样，逐步长出一棵或多棵树。

## 问题设定

地图是一个二维连续平面：

```text
start = 起点
goal  = 终点
rectangles = 矩形障碍物
```

算法每次随机采样一个点，然后从已有树中找最近节点，朝采样点前进一小步。如果这条边不碰撞，就把新节点加入树。

## 三种算法核心区别

| 算法 | 主要思想 | 优点 | 缺点 |
| --- | --- | --- | --- |
| RRT | 从起点长出一棵随机树，碰到目标附近就结束 | 简单、适合高维连续空间 | 路径通常不短，也不平滑 |
| RRT-Connect | 从起点和终点同时长两棵树，彼此尝试连接 | 找到可行路径通常更快 | 路径质量仍然不一定好 |
| RRT* | 在 RRT 基础上重选父节点并重连附近节点 | 迭代足够多时路径会逐渐优化 | 计算量更高，收敛需要时间 |

## 文件说明

```text
04_RRT Family/
├── README.md
├── requirements.txt
├── rrt_family_comparison.py
└── outputs/
    └── rrt_family_comparison.png
```

## 运行方式

在仓库根目录运行：

```powershell
python "04_RRT Family/rrt_family_comparison.py"
```

脚本会输出每种算法的：

- 是否找到路径
- 路径长度
- 树节点数量
- 运行时间

并生成对比图：

```text
04_RRT Family/outputs/rrt_family_comparison.png
```

如果缺少依赖，先安装：

```powershell
python -m pip install -r "04_RRT Family/requirements.txt"
```

## 图中看什么

- 灰色矩形：障碍物。
- 绿色点：起点。
- 红色点：终点。
- 浅蓝线：随机树扩展出来的边。
- 深蓝线：最终找到的路径。
- 底部柱状图：路径长度、树节点数量和运行时间对比。

## 一句话理解

- RRT：先找到一条能走的路。
- RRT-Connect：更快找到一条能走的路。
- RRT*：在能走的基础上，逐步把路变短。

## 一次典型运行结果

```text
Planner      Found       Length    Nodes   Time(ms)
RRT          True        246.62      374     285.86
RRT-Connect  True        158.25       87      22.27
RRT*         True        127.05     1224    3095.23
```

这个结果体现了 RRT family 的常见权衡：

- RRT 能找到可行路径，但路径质量通常较差。
- RRT-Connect 找路速度最快，路径也比普通 RRT 更直接。
- RRT* 路径最短，但节点更多、耗时更高。
