本系列旨在以我自己写的PID lib为例，讲一下PID的几点基本优化，PID的基本原理网上有很多资料，因此本系列将不会涉及PID的基本实现原理，在这里特别推荐Matlab tech talk的PID教程：https://ww2.mathworks.cn/videos/series/understanding-pid-control.html。

由于笔者大一在读，还没有学习自动控制原理等课程，因此本系列将不会从自控原理角度展开，相反的，本系列将试图**从“直觉”展开**，通过直观的描述让大家从直觉上感受并理解PID的一些包括微分先行、积分分离等基础的优化。

由于笔者水平有限，文中难免存在一些不足和错误之处，诚请各位批评指正。



（一）中主要讲解代码结构与代码使用，算法有关内容于（二）开始讲解

# 1 代码结构

该PID lib全部代码详见：https://github.com/CharlesW1970/PID_Library

## 1.1 PID结构体与有关枚举

该lib通过pid结构体保存于pid运算有关的参数数据，通过枚举表示其他有关量：

```c
//PID结构体
typedef struct _PID_TypeDef
{
    float Target;
    float LastNoneZeroTarget;
    float Kp;
    float Ki;
    float Kd;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;

    float Pout;
    float Iout; //Iout = ITerm_0 + ITerm_1 +....+ ITerm_n
    float Dout;
    float ITerm; //ITerm = Err * Ki

    float Output;
    float Last_Output;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ScalarA; //变积分公式参数
    float ScalarB; //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B

    uint8_t Improve; //用于使能优化

    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
        uint16_t maxOut,
        uint16_t integralLimit,
        float deadband,
        float Kp,
        float ki,
        float kd,
        float A,
        float B,
        uint8_t improve);

    void (*PID_reset)(
        struct _PID_TypeDef *pid,
        float Kp,
        float ki,
        float kd);
} PID_TypeDef;

//PID优化功能枚举
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //无
    Integral_Limit = 0x01,              //积分限幅
    Derivative_On_Measurement = 0x02,   //微分先行
    Trapezoid_Intergral = 0x04,         //梯形积分
    Proportional_On_Measurement = 0x08, //该系列不涉及
    OutputFilter = 0x10,                //输出滤波
    ChangingIntegralRate = 0x20,        //变积分
    ErrorHandle = 0x80,                 //异常处理
} PID_Improvement_e;

//异常情况枚举，这里只写了电机堵转保护一种
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

//异常情况结构体
typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;
```

## 1.2 PID初始化

在使用之前需要先调用PID_Init函数进行参数初始化和函数连接

```c
void PID_Init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float A,
    float B,

    uint8_t improve)
{
    pid->PID_param_init = f_PID_param_init;
    pid->PID_reset = f_PID_reset; //连接Kp Ki Kd参数重设函数
    pid->PID_param_init(pid, max_out, intergral_limit, deadband,
                        kp, Ki, Kd, A, B, improve); //连接并调用参数初始化函数
}

static void f_PID_param_init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float Changing_Integral_A,
    float Changing_Integral_B,

    uint8_t improve)
{
    //参数初始化
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->Target = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    pid->ScalarA = Changing_Integral_A;
    pid->ScalarB = Changing_Integral_B;

    pid->Improve = improve;

    //异常处理初始化
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}
```

## 1.3 PID 计算

PID_Calculate函数与网上大多数代码大体结构相同，只是添加了不同的优化函数，具体优化在各函数（如：f_PID_ErrorHandle、f_Trapezoid_Intergral）中实现，本系列除了会将与PID算法无关的异常处理放在最后，其他函数将按照在PID_Calculate函数中调用的先后顺序进行讲解，PID_Calculate函数具体代码如下：

```c
float PID_Calculate(PID_TypeDef *pid, float measure, float target)
{
    if (pid->Improve & ErrorHandle) 
    {
        //异常处理
        f_PID_ErrorHandle(pid);
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            //电机堵转保护
            pid->Output = 0;
            return 0; 
        }
    }

    //误差更新
    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    //死区内进行计算
    if (ABS(pid->Err) > pid->DeadBand)
    {
        //计算比例、微分输出与该周期积分项结果
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err);

        //判断是否使能梯形积分
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        //判断是否使能变积分
        if (pid->Improve & ChangingIntegralRate)
            f_Changing_Integral_Rate(pid);
        //判断是否使能积分限幅
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);
        //判断是否使能微分先行
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);

        //计算积分输出
        pid->Iout += pid->ITerm;

        //计算pid总输出
        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        //判断是否使能输出滤波
        if (pid->Improve & OutputFilter)
            f_OutputFilter(pid);

        //输出限幅
        f_Output_limit(pid);
    }
    //数据保存供下一周期调用
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Err = pid->Err;

    return pid->Output;
}
```

# 2 如何使用

这里给出以发布在GitHub上的示例，具体不在详细讲解

```c
//pid函数连接
PID_Init(&PID_Example, 9600, 5000, 3, 1, 5, 0.3, 0.3, 100, 100, 
   		 ErrorHandle | Integral_Limit | OutputFilter);

//修改kp ki kd
PID_Example.PID_reset(&PID_Example, 3, 1, 0);

//计算
PID_Calculate(&PID_Example, measure, target);
```

该篇对该lib结构和使用就讲到这里，下一篇将会开始算法讲解。