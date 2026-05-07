# Kalman Filter 学习项目

这个目录用一个二维目标跟踪实验对比三种常见 Kalman 滤波器：

- Kalman Filter, KF
- Extended Kalman Filter, EKF
- Unscented Kalman Filter, UKF

实验目标不是写一个最短实现，而是把每一步拆清楚，方便理解它们分别在解决什么问题。

## 问题设定

本目录现在包含两个对比实验：

- `kalman_filters_comparison.py`：温和非线性案例，主要演示 KF、EKF、UKF 的基础差异。
- `strong_nonlinear_ukf_comparison.py`：强非线性案例，重点展示 UKF 在非线性更强时的效果优势，以及它付出的计算时间成本。

### 案例一：温和非线性

我们跟踪一个在二维平面运动的目标，状态向量为：

```text
x = [px, py, vx, vy]
```

含义：

- `px, py`：目标位置
- `vx, vy`：目标速度

系统运动模型使用近似匀速模型：

```text
x_k = F x_{k-1} + process_noise
```

传感器观测不是直接给出 `px, py`，而是给出非线性的极坐标观测：

```text
z = [range, bearing]
```

含义：

- `range = sqrt(px^2 + py^2)`
- `bearing = atan2(py, px)`

这个设定适合学习三种滤波器的差别，因为运动模型是线性的，但观测模型是非线性的。

### 案例二：强非线性

强非线性案例使用更接近雷达/定位融合的问题：

```text
x = [px, py, speed, heading, turn_rate]
```

运动模型是 coordinated turn，也就是带转弯率的非线性运动：

```text
px, py 会随 speed、heading、turn_rate 非线性变化
```

观测模型为：

```text
z = [range_from_origin, bearing_from_origin, range_from_landmark]
```

这里同时有原点距离、原点方位角和到固定地标的距离。这个观测函数比单一极坐标观测更弯曲，EKF 的一阶线性化更容易损失信息，UKF 的 sigma points 更容易体现优势。

## 三种方法的核心区别

| 方法 | 适合场景 | 如何处理非线性 | 优点 | 缺点 |
| --- | --- | --- | --- | --- |
| KF | 状态转移和观测都线性 | 不直接处理非线性；本项目中先把极坐标观测转换成 x/y | 最简单、速度快、公式清晰 | 非线性观测下会引入转换误差 |
| EKF | 弱到中等非线性系统 | 用一阶泰勒展开，把非线性函数局部线性化 | 工程中常见，计算量适中 | 需要推导雅可比矩阵；非线性强时可能不稳定 |
| UKF | 中等到较强非线性系统 | 用 sigma points 传播均值和协方差 | 不需要手写雅可比；通常比 EKF 更准确 | 计算量比 EKF 大；参数需要理解 |

## 文件说明

```text
02_Kalman Filter/
├── README.md
├── kalman_filters_comparison.py
├── strong_nonlinear_ukf_comparison.py
├── requirements.txt
└── outputs/
    ├── kalman_filters_comparison.png
    └── strong_nonlinear_ukf_comparison.png
```

## 运行方式

在仓库根目录运行：

```powershell
python "02_Kalman Filter/kalman_filters_comparison.py"
```

强非线性案例：

```powershell
python "02_Kalman Filter/strong_nonlinear_ukf_comparison.py"
```

脚本会输出每种方法的 RMSE 和运行时间，并生成对比图：

```text
02_Kalman Filter/outputs/kalman_filters_comparison.png
02_Kalman Filter/outputs/strong_nonlinear_ukf_comparison.png
```

如果缺少依赖，先安装：

```powershell
python -m pip install -r "02_Kalman Filter/requirements.txt"
```

## 建议学习顺序

1. 先看 `simulate_data()`：理解真实轨迹和带噪声观测是怎么生成的。
2. 再看 `KalmanFilter2D`：掌握预测 `predict` 和更新 `update` 的基本结构。
3. 再看 `ExtendedKalmanFilter2D`：重点理解观测函数 `h_polar()` 和雅可比 `jacobian_h_polar()`。
4. 最后看 `UnscentedKalmanFilter2D`：重点理解 sigma points 如何绕过手写雅可比。
5. 对照输出图看误差曲线，思考为什么 KF 在非线性观测转换后通常更吃亏。
6. 再运行 `strong_nonlinear_ukf_comparison.py`，观察 UKF 为什么能在强非线性下取得更低 RMSE，以及为什么计算时间更高。

## 图中看什么

- `Trajectory`: 真实轨迹、带噪声观测、三种估计轨迹。
- `Position Error`: 每个时刻的位置误差，越低越好。
- `RMSE`: 整体均方根误差，越低越好。
- `Final Estimate`: 最后时刻三种方法的估计位置和真实位置对比。
- `Cost`: 强非线性图中的平均运行时间，体现效果和计算成本之间的权衡。

## 强非线性案例的一次典型结果

```text
 KF RMSE: 6.150 | avg run:  2.797 ms
EKF RMSE: 4.477 | avg run: 18.689 ms
UKF RMSE: 3.968 | avg run: 37.871 ms
```

这个结果说明：

- KF 最快，但模型假设最弱，强非线性下误差最大。
- EKF 比 KF 准很多，但依赖局部线性化。
- UKF 在这个强非线性案例里 RMSE 最低，但计算时间最高。
