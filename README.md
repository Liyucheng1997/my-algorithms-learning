# 我的算法学习

这个仓库用于记录我用 Python 学习算法的过程。内容会从可视化、可运行的小实验开始，逐步扩展到更完整的理论笔记、代码实现和实验结果。

## 学习方向

- 强化学习：Q-Learning、策略迭代、值迭代、Deep Q-Network 等。
- 状态估计：Kalman Filter、Extended Kalman Filter、Particle Filter 等。
- 神经网络：感知机、多层感知机、反向传播、CNN、RNN、Transformer 基础等。
- 优化算法：梯度下降、随机梯度下降、Adam、牛顿法等。
- 数值计算：矩阵运算、概率分布、采样与仿真。

## 目录结构

```text
.
├── 01_Mouse Q-Learning/
│   ├── Q-learning_on_Maze.py
│   └── training_result.png
├── 02_Kalman Filter/
├── README.md
└── .gitignore
```

## 当前内容

### 01_Mouse Q-Learning

一个用 Q-Learning 训练小鼠走迷宫的实验，包含 Python 代码和训练结果图片。

### 02_Kalman Filter

预留给 Kalman 滤波器学习内容。建议后续补充：

- `notes.md`：公式推导和直觉解释。
- `kalman_1d.py`：一维位置估计示例。
- `kalman_2d.py`：二维运动轨迹估计示例。
- `README.md`：本主题的实验说明。

## 推荐学习记录格式

每个主题目录可以尽量保持下面的结构：

```text
主题目录/
├── README.md        # 本主题简介、运行方式、学习总结
├── notes.md         # 理论笔记
├── src/             # Python 实现
├── notebooks/       # Jupyter Notebook，可选
├── data/            # 小型示例数据，可选
└── outputs/         # 图片、图表、实验结果
```

## Python 环境建议

推荐使用虚拟环境，避免不同实验之间的依赖互相影响。

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
```

如果某个主题需要第三方库，可以在仓库根目录或主题目录维护 `requirements.txt`。

常用依赖示例：

```text
numpy
matplotlib
pandas
scipy
jupyter
```

## Git 使用

常用命令：

```powershell
git status
git add .
git commit -m "初始化算法学习仓库"
git branch -M main
git remote add origin <你的 GitHub 仓库地址>
git push -u origin main
```

后续每完成一个小实验，可以按下面的节奏提交：

```powershell
git add .
git commit -m "添加 Kalman Filter 一维示例"
git push
```

## 学习原则

- 每个算法都尽量包含：问题背景、核心公式、直觉解释、Python 实现、可视化结果和总结。
- 先写最小可运行版本，再逐步优化结构。
- 重要实验结果保留图片或日志，方便回顾。
- 代码以清晰为主，性能优化放在理解之后。
