#### 1 构建模型

**过程模型：**
$$
\mathbf{x}_{k}=\mathbf{F}_{k} \mathbf{x}_{k-1}+B_k\mathbf{u}_{k-1}+\Gamma_{k-1} \mathbf{w}_{k-1}, \quad \mathbf{w}_{k} \sim \mathscr{N}\left(\mathbf{0}_{n \times 1}, \mathbf{Q}_{k}\right)
$$
**观测模型：**
$$
\mathbf{z}_{k}=\mathbf{H}_{k} \mathbf{x}_{k}+\mathbf{v}_{k}, \mathbf{v}_{k} \sim \mathscr{N}\left(\mathbf{0}_{m \times 1}, \mathbf{R}_{k}\right)
$$
其中，$w_k$ 和 $v_k$ 是互不相关的高斯白噪声，他们的噪声方差矩阵分别为 $Q_k$ 和 $R_k$ 。



#### 2 状态初始化

$$
\left\{\begin{array}{l}
\hat{\mathbf{x}}_{0}=\mathrm{E}\left(\mathbf{x}_{0}\right) \\
\mathbf{P}_{0}=\mathrm{E}\left[\left(\mathbf{x}_{0}-\mathrm{E}\left(\mathbf{x}_{0}\right)\right)\left(\mathbf{x}_{0}-\mathrm{E}\left(\mathbf{x}_{0}\right)\right)^{\mathrm{T}}\right]
\end{array}\right.
$$

当 $k=0$ 时，取 $\mathbf{P}_{0 | 0}=\mathbf{P}_{0}, \hat{\mathbf{x}}_{0 | 0}=\hat{\mathbf{x}}_{0}$。



#### 3 状态估计预测

$$
\hat{\mathbf{x}}_{k | k-1}=\mathbf{F}_{k} \hat{\mathbf{x}}_{k-1 | k-1}+B_k\mathbf{u}_{k-1}
$$



#### 4 误差协方差预测

$$
\mathbf{P}_{k | k-1}=F_{k} \mathbf{P}_{k-1 | k-1} \mathbf{F}_{k}^{T}+\Gamma_{k-1} \mathbf{Q}_{k-1} \Gamma_{k-1}^{T}
$$



#### 5 卡尔曼增益更新

$$
\mathbf{K}_{k}=\mathbf{P}_{k | k-1} \mathbf{H}_{k}^{\mathrm{T}}\left(\mathbf{H}_{k} \mathbf{P}_{k | k-1} \mathbf{H}_{k}^{\mathrm{T}}+\mathbf{R}_{k}\right)^{-1}
$$



#### 6 状态估计更新

$$
\hat{\mathbf{x}}_{k | k}=\hat{\mathbf{x}}_{k | k-1}+\mathbf{K}_{k}\left(\mathbf{z}_{k}-\hat{\mathbf{z}}_{k | k-1}\right)
$$

其中，$\mathbf{z}_{k | k-1}=\mathbf{H}_{k} \hat{\mathbf{x}}_{k | k-1}$。



#### 7 协方差矩阵更新

$$
\mathbf{P}_{k | k}=\left(\mathbf{I}_{n}-\mathbf{K}_{k} \mathbf{H}_{k}\right) \mathbf{P}_{k | k-1}
$$

