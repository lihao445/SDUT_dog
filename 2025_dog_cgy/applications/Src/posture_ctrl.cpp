#include "posture_ctrl.h"
#include <math.h>

GaitParams gait_params={-259.8f, 100.0f, 30.0f,
     0.5f,1.0f}; // 初始腿部参数
GaitParams state_gait_params[] ={  // 不同状态下的参数设置
    //站立高度，步长，抬腿高度，飞行百分比，频率
    {-259.8f, 100.0f, 30.0f, 0.5f, 1.0f}
 }; 

DetachedParam detached_params;
DetachedParam state_detached_params[] = {
    // 站立高度，步长，抬腿高度，飞行百分比，频率
    {
        {-259.8f, 100.0f, 30.0f, 0.5f, 1.0f},
        {-259.8f, 100.0f, 30.0f, 0.5f, 1.0f},
        {-259.8f, 100.0f, 30.0f, 0.5f, 1.0f},
        {-259.8f, 100.0f, 30.0f, 0.5f, 1.0f}
    },
    {
        {-259.8f, 0.0f, 0.0f, 0.5f, 1.0f},
        {-259.8f, 0.0f, 0.0f, 0.5f, 1.0f},
        {-259.8f, 0.0f, 0.0f, 0.5f, 1.0f},
        {-259.8f, 0.0f, 0.0f, 0.5f, 1.0f}
    }
};

bool _leg_active[4]= {1,1,1,1};

enum States state = TROT; // 初始状态
 
extern "C"
 /**
 * @brief 姿态控制任务函数，根据机器人当前状态执行相应的步态控制
 * 
 * @param pvParameters 任务创建时传递的参数，在本函数中未使用
 */
 void PostureControl_task(void *pvParameters)
{
    for(;;)
    {
        switch(state)
        {
            gait_params = state_gait_params[state];
            detached_params = state_detached_params[state];

            case TROT:
            gait_detached(detached_params,
                0.0f, 0.5f, 0.0f, 0.5f,
                1.0f, 1.0f, 1.0f, 1.0f);
                break;
            case START:
            motor_init();
                break;
            case REALSE:
            vTaskDelay(500);
                break;
            case STOP:
            gait_detached(detached_params,
                0.0f, 0.5f, 0.0f, 0.5f,
                1.0f, 1.0f, 1.0f, 1.0f);
                break;
            default:
                break;
        }
        vTaskDelay(10);
    }
}

/**
 * @brief 分离式步态控制函数，根据腿部激活状态和参数控制每条腿的运动
 * 
 * @param d_params 分离式步态参数结构体，包含每条腿的独立参数
 * @param leg0_offset 腿部0的步态偏移量
 * @param leg1_offset 腿部1的步态偏移量
 * @param leg2_offset 腿部2的步态偏移量
 * @param leg3_offset 腿部3的步态偏移量
 * @param leg0_direction 腿部0的运动方向
 * @param leg1_direction 腿部1的运动方向
 * @param leg2_direction 腿部2的运动方向
 * @param leg3_direction 腿部3的运动方向
 */
void gait_detached(	DetachedParam d_params,
    float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
    float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction) {

    // 获取当前时间（单位：秒）
    float t = HAL_GetTick()/1000.0f;

    // 检查腿部0是否激活，如果激活则调用耦合运动函数控制腿部运动
    if(_leg_active[0] == true)
        CoupledMoveLeg( t, d_params.detached_params_0, 
    leg0_offset, leg0_direction, 0);

    // 检查腿部1是否激活，如果激活则调用耦合运动函数控制腿部运动
    if(_leg_active[1] == true) 
        CoupledMoveLeg( t, d_params.detached_params_1, 
    leg1_offset, leg1_direction, 1);

    // 检查腿部2是否激活，如果激活则调用耦合运动函数控制腿部运动
    if(_leg_active[2] == true)
        CoupledMoveLeg( t, d_params.detached_params_2, 
    leg2_offset, leg2_direction, 2);

    // 检查腿部3是否激活，如果激活则调用耦合运动函数控制腿部运动
    if(_leg_active[3] == true)
        CoupledMoveLeg( t, d_params.detached_params_3, 
    leg3_offset, leg3_direction, 3);

}

/**
 * @brief 耦合腿部运动控制函数，根据给定的时间、步态参数、步态偏移量、腿部方向和腿部ID控制腿部运动
 * 
 * @param t 当前时间（单位：秒），用于计算轨迹
 * @param params 步态参数结构体，包含站立高度、步长、抬腿高度等信息
 * @param gait_offset 步态偏移量，用于调整轨迹的起始位置
 * @param leg_direction 腿部运动方向，影响逆运动学计算结果
 * @param LegId 腿部ID，用于指定要控制的腿部
 */
void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId){
    // 定义变量用于存储腿部的目标位置坐标
    float x, y;
    // 调用贝塞尔轨迹函数，根据当前时间、步态参数和步态偏移量计算腿部的目标位置坐标
    beziertraectory(t, params,gait_offset, &x, &y);
    // 定义变量用于存储腿部关节的目标角度
    float theta1, theta2;
    // 调用逆运动学函数，根据腿部的目标位置坐标和腿部方向计算腿部关节的目标角度
    InverseKinematics(x, y, &theta1, &theta2,leg_direction);
    // 调用设置耦合位置函数，根据腿部ID和腿部关节的目标角度设置电机的参考位置
    SetCoupledPosition(LegId,theta1,theta2);  
}

/**
 * @brief 计算贝塞尔轨迹，根据给定的时间、步态参数和步态偏移量计算腿部的目标位置坐标
 * 
 * @param t 当前时间（单位：秒），用于计算轨迹
 * @param params 步态参数结构体，包含站立高度、步长、抬腿高度等信息
 * @param gaitoffest 步态偏移量，用于调整轨迹的起始位置
 * @param x 指向存储计算得到的X轴坐标的变量的指针
 * @param y 指向存储计算得到的Y轴坐标的变量的指针
 */
void beziertraectory(float t, GaitParams params,float gaitoffest,float *x, float *y) {
    // 静态变量p用于累计时间，p_last用于记录上一次的时间
    static float p=0;
    static float p_last=0;

    // 从步态参数结构体中提取所需参数
    float stanceHeight = params.stance_height;
    float liftHeight = params.lift_height;
    float flightPercent = params.flight_percent;
    float stepLength = params.step_length;
    float FREQ = params.freq;

    // 更新p的值，仅在时间差小于0.5秒时累加
    p+=FREQ*(t-p_last<0.5f?t-p_last:0);
    // 更新上一次的时间
    p_last=t;

    // 通过频率计算归一化时间（替代原CYCLE_TIME方案），确保时间在[0, 1)范围内
    float t_norm = fmodf(p+gaitoffest, 1.0f); 

    // 摆动相处理（飞行阶段）
    if (t_norm <= flightPercent) {
        // 计算摆动相的归一化时间
        float t_swing = t_norm / flightPercent;
        // 计算t的各次幂，用于后续多项式计算
        float t1 = t_swing;
        float t2 = t1 * t1;
        float t3 = t2 * t1;
        float t4 = t3 * t1;
        float t5 = t4 * t1;
        // 计算X轴五次多项式轨迹，根据多项式公式计算腿部在X轴的位置
        // 原式：10*t3*(1-t_swing)^2 + 5*t4*(1-t_swing) + t5
        *x = -stepLength/2 + stepLength * (
            t3*(10 - 20*t1 + 10*t2) + 
            t4*(5 - 5*t1) + 
            t5
        );
        // Y轴分两阶段处理
        if (t_norm <= flightPercent * 0.5f) { // 抬腿阶段
            // 计算抬腿阶段的局部归一化时间
            float t_local = t_norm / (flightPercent * 0.5f);
            // 根据多项式公式计算腿部在Y轴的位置
            *y = stanceHeight + liftHeight * 
            (10*powf(t_local,3)*powf(1-t_local,2) + 5*powf(t_local,4)*(1-t_local) + powf(t_local,5));
        } else { // 放腿阶段
            // 计算放腿阶段的局部归一化时间
            float t_local = (t_norm - flightPercent*0.5f) / (flightPercent*0.5f);
            // 根据多项式公式计算腿部在Y轴的位置
            *y = stanceHeight + liftHeight * 
            (10*powf(1-t_local,3)*powf(t_local,2) + 5*powf(1-t_local,4)*t_local + powf(1-t_local,5));
        }
    } 
    // 支撑相处理
    else {
        // 计算支撑相的归一化时间
        float t_support = (t_norm - flightPercent) / (1 - flightPercent);
        // 计算腿部在X轴的位置
        *x = stepLength/2 - stepLength * t_support;
        // 腿部在Y轴的位置保持为站立高度
        *y = stanceHeight; 
    }
}

// 快速平方根函数，使用了牛顿迭代法来计算平方根的倒数
float Q_rsqrt(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = number * 0.5f;
    y = number;
    i = *(long*)&y;
    i = 0x5F3759DF - (i >> 1);
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y)); // 一次牛顿迭代
    return y;
}

// 逆运动学（输入x,y，输出theta1和theta2，单位：弧度）
/**
 * @brief 逆运动学计算函数，根据腿部的目标位置坐标和腿部方向计算腿部关节的目标角度
 * 
 * 该函数接收腿部的目标位置坐标 (x, y)、腿部方向以及存储关节角度的指针，
 * 通过逆运动学计算得出两个关节的角度 theta1 和 theta2。
 * 
 * @param x 腿部目标位置的 X 轴坐标
 * @param y 腿部目标位置的 Y 轴坐标
 * @param theta1 指向存储第一个关节角度（弧度）的变量的指针
 * @param theta2 指向存储第二个关节角度（弧度）的变量的指针
 * @param leg_direction 腿部运动方向，1 表示正向，-1 表示反向
 */
void InverseKinematics(float x, float y, float *theta1, float *theta2,float leg_direction) {
    // 计算目标位置到原点的距离的平方
    float L_sq = x*x + y*y;
    // 使用快速平方根算法计算目标位置到原点的距离
    float L = 1.0f / Q_rsqrt(L_sq); 
    // 计算余弦定理中的分子部分
    float numerator = L_sq + L1*L1 - L2*L2;
    // 计算余弦定理中的分母部分
    float denominator = 2.0f * L1 * L;
    // 计算角度 beta 的余弦值
    float cos_beta = numerator / denominator;
    
    // 限制 cos_beta 的值在 [-1, 1] 范围内，避免 acosf 函数出现定义域错误
    cos_beta = (cos_beta > 1.0f) ? 1.0f : (cos_beta < -1.0f) ? -1.0f : cos_beta;
    // 计算角度 beta
    float beta = acosf(cos_beta);
    // 计算目标位置向量与 X 轴正方向的夹角
    float phi = atan2f(y, x);

    // 根据腿部运动方向计算关节角度
    if(leg_direction == 1){
        // 正向运动时，计算第一个关节的角度
        *theta1 = phi - beta + M_PI;
        // 正向运动时，计算第二个关节的角度
        *theta2 = phi + beta;
    }
    else if(leg_direction == -1){
        // 反向运动时，计算第二个关节的角度
        *theta2 = -(phi - beta + M_PI);
        // 反向运动时，计算第一个关节的角度
        *theta1 = -(phi + beta);
    }
}

float ThetaToEncoder(float theta_rad) {
    float motor_angle_rad = theta_rad * 19.0f;
    return  (motor_angle_rad * 8191) / (2.0f * M_PI);
}

/**
 * @brief 设置耦合位置函数，根据腿部ID和关节角度设置电机的参考位置，并更新电机就绪状态
 * 
 * 该函数根据传入的腿部ID，将对应的关节角度 theta1 和 theta2 转换为编码器值，
 * 并存储到临时变量 temp 中。同时，将电机就绪状态更新为 IsReady。
 * 
 * @param LegId 腿部ID，取值范围为 0 到 3，分别代表四条腿
 * @param theta1 第一个关节的角度（弧度）
 * @param theta2 第二个关节的角度（弧度）
 */
void SetCoupledPosition( int LegId,float theta1, float theta2) {
    // 根据不同的腿部ID，将关节角度转换为编码器值并存储到 temp 变量中
    if(LegId==0){
        // 将腿部 0 的第一个关节角度转换为编码器值并存储
        temp.ref_theta1[0] = ThetaToEncoder(theta1);
        // 将腿部 0 的第二个关节角度转换为编码器值并存储
        temp.ref_theta2[0] = ThetaToEncoder(theta2);
    }
    else if(LegId==1){
        temp.ref_theta1[1] = ThetaToEncoder(theta1);
        temp.ref_theta2[1] = ThetaToEncoder(theta2);
    }
    else if(LegId==2){
        temp.ref_theta1[2] = ThetaToEncoder(theta1);
        temp.ref_theta2[2] = ThetaToEncoder(theta2);
    }
    else if(LegId==3){
        temp.ref_theta1[3] = ThetaToEncoder(theta1);
        temp.ref_theta2[3] = ThetaToEncoder(theta2);
    }
    // 更新电机就绪状态
    IsMotoReadyOrNot = IsReady;
}