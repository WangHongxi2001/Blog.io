在[Mahony姿态解算算法笔记（一）](https://www.cnblogs.com/HongxiWong/p/12357230.html)中，我们介绍了融合加速度计与陀螺仪共六轴数据的姿态解算算法，该篇将介绍融合加速度计、陀螺仪和磁力计共九轴数据的姿态解算算法。

该算法可到其网站下载源码https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

由于笔者水平有限，文中难免存在一些不足和错误之处，诚请各位批评指正。

# 1 引入磁力计数据的目的

在六轴数据融合姿态解算算法中，我们通过理论和实际重力加速度向量来补偿陀螺仪误差，这种方法存在的一个问题就在**重力加速度向量垂直于大地坐标系的$xoy$平面**，因此在飞机水平时，重力加速度无法反应机体Yaw轴的角度变化，因此无法很有效的修正Yaw轴的角度数据，因此我们需要引入磁力计数据。由于地球磁场方向在中低纬度地区与地面大致平行，因此通过磁力计数据我们可以有效的修正Yaw轴的角度数据。

但在一部分情况中，复杂的磁场环境会导致九轴数据融合的结果并不会优于六轴数据太多，甚至劣于六轴数据融合的结果，比如Robomaster机器人云台或使用无刷电机的Mini四轴的姿态解算，有相当一部分参赛队会选择融合六轴数据而非九轴数据。原因是云台复杂多变的磁场环境会给磁力计引入很大的噪声，这就导致了有些情况下九轴数据的姿态融合结果并不会比六轴好。

综上所述，具体选择九轴还是六轴数据的融合还需要根据具体环境来决定。



# 2 准备工作

在姿态解算过程中，**传感器的数据并非直接输入该算法中**，在使用Mahony算法，我们需要对加速度计和磁力计的数据进行**滤波**（很重要）和**校正**（没那么重要）

## 2.1 传感器滤波

机体的震动会给加速度计数据带来严重的高频噪声。因此，对于加速度计数据的低通滤波是很有必要的。一般情况下，对加速度数据进行低通滤波即可满足控制要求，根据实际情况可灵活选用滤波算法。

## 2.2 传感器校正

由于 $x,y,z$ 三轴的单位存在差异，测量数据的向量顶点会落在椭球面上而非理想状态下的球面上。另外的，加速度计于磁力计测量的数据会存在一定的偏移，这则会导致椭球的中心不一定在原点，这也就是数据的偏移。因此，在对于精度要求高的场合下我们也需要对传感器数据进行校正。

一般来说，加速度计和磁力计的校正，需要求出椭球的**球心**即偏移量，椭球**轴长**，即椭球拟合。具体椭球拟合算法可以参考https://blog.csdn.net/shenshikexmu/article/details/70143455



数据经过以上步骤以后就可以进入Mahony算法中了。



# 3 融合磁力计数据

九轴数据融合于六轴数据融合算法的思路与框架是完全一致的，即九轴数据融合的实现只需在六轴数据融合算法的基础上加入磁力计修正部分即可，简单来说就是添几行代码。

与通过加速度计数据补偿陀螺仪大致相同，**通过磁力计数据和姿态矩阵得到理论磁场向量和实际磁场向量，并通过向量外积得到向量偏差，最后将偏差带入的PI控制器计算陀螺仪数据补偿值。**

唯一的取别在于标准重力加速度在星球表面任何地方始终固定不变，可直接从地理坐标系变换到机体坐标系，但地磁的大小与方向并非固定不变，**因此需要经测量得到，但由于磁力计固连在机体坐标系，因此通过三轴磁力计得到的磁场方向向量是以机体坐标系 $\hat{\boldsymbol{x}}'$  $\hat{\boldsymbol{y}}'$  $\hat{\boldsymbol{z}}'$ 为基表示的，因此该向量还需要通过 $C_{b}^{R}$ 转换得到地理坐标系中的磁力向量，然后再将其通过 $C_{R}^{b}$ 转换回机体坐标系，从而得到理论磁力向量。**

关于下列姿态矩阵详见[Mahony姿态解算算法笔记（一）](https://www.cnblogs.com/HongxiWong/p/12357230.html)

$C_{b}^{R}$ （从机体坐标系转换至地理坐标系）：
$$
C_{b}^{R}=\left[\begin{array}{lll}
{1-2\left(q_{2}^{2}+q_{3}^{2}\right)} & {2\left(q_{1} q_{2}-q_{0} q_{3}\right)} & {2\left(q_{1} q_{3}+q_{0} q_{2}\right)} \\
{2\left(q_{1} q_{2}+q_{0} q_{3}\right)} & {1-2\left(q_{1}^{2}+q_{3}^{2}\right)} & {2\left(q_{2} q_{3}-q_{0} q_{1}\right)} \\
{2\left(q_{1} q_{3}-q_{0} q_{2}\right)} & {2\left(q_{2} q_{3}+q_{0} q_{1}\right)} & {1-2\left(q_{1}^{2}+q_{2}^{2}\right)}
\end{array}\right]
$$
$C_{R}^{b}$（从地理坐标系转换至机体坐标系）：
$$
C_{R}^{b}=\left[\begin{array}{lll}
{1-2\left(q_{2}^{2}+q_{3}^{2}\right)} & {2\left(q_{1} q_{2}+q_{0} q_{3}\right)} & {2\left(q_{1} q_{3}-q_{0} q_{2}\right)} \\
{2\left(q_{1} q_{2}-q_{0} q_{3}\right)} & {1-2\left(q_{1}^{2}+q_{3}^{2}\right)} & {2\left(q_{2} q_{3}+q_{0} q_{1}\right)} \\
{2\left(q_{1} q_{3}+q_{0} q_{2}\right)} & {2\left(q_{2} q_{3}-q_{0} q_{1}\right)} & {1-2\left(q_{1}^{2}+q_{2}^{2}\right)}
\end{array}\right]
$$

## 3.1 将磁场信息表示为向量

地球磁场可以分解为（投影到）水平和竖直两个分量，其中竖直分量与水平分量的比值由磁场与大地（局部来看水平）的倾角决定，而这个倾角在不同纬度地区会有所不同，在赤道地区磁场方向就与大地几乎平行，而在磁极处则垂直于大地，因此在两磁极处磁力计就无法发挥作用了。

因此我们可以将由测量和姿态矩阵转换得到的**理论磁场向量** $\hat{\boldsymbol{m}}$ 与仅通过测量得到的**实际磁场向量** $\overline{\boldsymbol{m}}$ 表示为：
$$
\hat{\boldsymbol{m}}=\left[\begin{array}{lll}
b_{x}\\
0 \\ 
b_{z}
\end{array}\right] \\
\overline{\boldsymbol{m}}=\left[\begin{array}{lll}
m_{x} \\
m_{y} \\
m_{z}
\end{array}\right]
$$

## 3.2 如何得到两个磁场向量

实际磁场向量 $\overline{\boldsymbol{m}}$ 可通过三轴磁力计直接得到。但考虑到由测量得到的磁场向量还需要经过矩阵变换等操作才能最终得到机体坐标系下的理论磁场向量 $\hat{\boldsymbol{m}}$ 。因此我们需要引入一个中间向量  $\hat{\boldsymbol{h}}$ ：
$$
\hat{\boldsymbol{h}}=\left[\begin{array}{l}
{h_x} \\
{h_y} \\
{h_z} 
\end{array}\right]
$$
我们需要将实际磁场向量 $\overline{\boldsymbol{m}}$ 通过 $C_{b}^{R}$ 转换得到地理坐标系下的磁场向量  $\hat{\boldsymbol{h}}$ ，两者的转换公式可通过前文给出的矩阵 $C_{b}^{R}$ 直接转换得出
$$
\hat{\boldsymbol{h}}=C_{b}^{R} \cdot \overline{\boldsymbol{m}}=\left[\begin{array}{lll}
{1-2\left(q_{2}^{2}+q_{3}^{2}\right)} & {2\left(q_{1} q_{2}-q_{0} q_{3}\right)} & {2\left(q_{1} q_{3}+q_{0} q_{2}\right)} \\
{2\left(q_{1} q_{2}+q_{0} q_{3}\right)} & {1-2\left(q_{1}^{2}+q_{3}^{2}\right)} & {2\left(q_{2} q_{3}-q_{0} q_{1}\right)} \\
{2\left(q_{1} q_{3}-q_{0} q_{2}\right)} & {2\left(q_{2} q_{3}+q_{0} q_{1}\right)} & {1-2\left(q_{1}^{2}+q_{2}^{2}\right)}
\end{array}\right]\cdot
\left[\begin{array}{lll}
m_{x} \\
m_{y} \\
m_{z}
\end{array}\right] =\left[\begin{array}{l}
{h_x} \\
{h_y} \\
{h_z} 
\end{array}\right]
$$
通过 **3.1** 中提到的对磁场的分解，我们了解到，只需要水平和竖直两个分量即可表示磁场向量，而地理坐标系中的 $x$ 轴与 $y$ 轴均为水平轴，因此我们可以将 $\hat{\boldsymbol{h}}$ 中 $x$ 轴与 $y$ 轴的两个分量合并得到 $b_x$ ，而竖直分量 $h_z$ 保持不变，由此即可求出 $\hat{\boldsymbol{m}}$ ，即：
$$
\hat{\boldsymbol{m}}=\left[\begin{array}{lll}
b_{x}\\
0 \\ 
b_{z}
\end{array}\right]
 = 
 \left[\begin{array}{lll}
 \sqrt{h_{x}^{2}+h_{y}^{2}}\\
 0\\
 h_z
 \end{array}\right]
$$
现在可以通过向量外积计算理论向量与实际向量的误差了吗？**还不行**，因为我们的实际磁场向量 $\overline{\boldsymbol{m}}$ 以机体坐标系 $\hat{\boldsymbol{x}}'$  $\hat{\boldsymbol{y}}'$  $\hat{\boldsymbol{z}}'$ 为基，而刚刚计算出的理论磁场向量 $\hat{\boldsymbol{m}}$ 是以地理坐标系的 $\hat{\boldsymbol{x}}$  $\hat{\boldsymbol{y}}$  $\hat{\boldsymbol{z}}$ 为基的，因此我们还需要将 $\hat{\boldsymbol{m}}$ 通过矩阵 $C_{R}^{b}$ 转换到机体坐标系：
$$
\boldsymbol{\hat{m’}}=
C_{R}^{b}\cdot\hat{\boldsymbol{m}}=
\left[\begin{array}{lll}
{1-2\left(q_{2}^{2}+q_{3}^{2}\right)} & {2\left(q_{1} q_{2}+q_{0} q_{3}\right)} & {2\left(q_{1} q_{3}-q_{0} q_{2}\right)} \\
{2\left(q_{1} q_{2}-q_{0} q_{3}\right)} & {1-2\left(q_{1}^{2}+q_{3}^{2}\right)} & {2\left(q_{2} q_{3}+q_{0} q_{1}\right)} \\
{2\left(q_{1} q_{3}+q_{0} q_{2}\right)} & {2\left(q_{2} q_{3}-q_{0} q_{1}\right)} & {1-2\left(q_{1}^{2}+q_{2}^{2}\right)}
\end{array}\right]
\cdot
\left[\begin{array}{lll}
b_{x}\\
0 \\ 
b_{z}
\end{array}\right]
=
\left[\begin{array}{lll}
b_{x}^{'}\\
0 \\ 
b_{z}^{'}
\end{array}\right]
$$

## 3.3 误差补偿

经过上述步骤，我们已经得到了同为机体坐标系的两个磁场向量，接下来只需要通过求两个向量外积得到误差，并与由加速度计得到的误差相加即可：
$$
|\boldsymbol{\rho}|=|\overline{\boldsymbol{v}}| \cdot|\hat{\boldsymbol{v}}| \cdot \sin \theta_1 +|\overline{\boldsymbol{v}}| \cdot|\hat{\boldsymbol{v}}| \cdot \sin \theta_2
$$
注意，与两个重力加速度向量相同，在外积运算前也需要进行**单位化**处理，因此上式可简化为：
$$
|\boldsymbol{\rho}|=\sin \theta_1 +\sin \theta_2
$$
由于理论向量和实际向量偏差角不会超过45°，而当 *θ* 在±45°内时，sin*θ* 与 *θ*的值非常接近，因此上式可进一步简化为：
$$
|\boldsymbol{\rho}|=\theta_1 +\theta_2
$$
这样一来，我们就得到了融合了加速度计和磁力计显化出的陀螺仪误差，接下来就可以通过构建PI控制器来控制补偿值的大小了，由于这里与[Mahony姿态解算算法笔记（一）](https://www.cnblogs.com/HongxiWong/p/12357230.html)中完全一致，这里不再赘述。

## 3.4 代码分析

```c
// Definitions
#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain


// Variable definitions
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, hz, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    // 在磁力计数据无效时使用六轴融合算法
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // 只在加速度计数据有效时才进行运算
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
        // 将加速度计得到的实际重力加速度向量v单位化
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
        // 将磁力计得到的实际磁场向量m单位化
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        // 辅助变量，以避免重复运算
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        // 通过磁力计数据与坐标转换矩阵得到理论磁场向量
        // 公式（5）
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        // 公式（6）
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
        // 将理论重力加速度向量与理论磁场向量通过坐标转换矩阵转换至机体坐标系
        // 注意，这里实际上是矩阵*1/2，在开头对Kp Ki的宏定义均为2*增益
        // 至于这么设计程序的原因笔者也没有弄清，猜测可能是为减少乘法运算量？
       	// 笔记（一）中内容
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        // 公式（7）
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
        // 通过向量外积得到重力加速度向量和磁场向量的实际值与测量值直接误差
        // 公式（8）（9）（10）
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
        // 在误差补偿PI控制器中积分项使能情况下计算并应用积分项
		if(twoKi > 0.0f) {
            // integral error scaled by Ki
            // 积分过程
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            
            // apply integral feedback
            // 应用误差补偿中的积分项
			gx += integralFBx;	
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
            // prevent integral windup
            // 避免为负值的Ki时积分异常饱和
			integralFBx = 0.0f;	
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
        // 应用误差补偿中的比例项
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
    //一阶龙格库塔法迭代四元数
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
    // 单位化四元数 保证四元数在迭代过程中保持单位性质
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
    
    //Mahony官方程序到此结束，使用时只需在函数外进行四元数反解欧拉角即可完成全部姿态解算过程
}
```

