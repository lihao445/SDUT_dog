#include "motion_clac.h"
#include <cmath>
#include <stdexcept>

using namespace std;

/**
 * @brief 实现机器人的前向运动学计算，即根据关节角度计算出末端执行器的位置。
 * 
 * @param theta1 第一个关节的角度
 * @param theta2 第二个关节的角度
 * @return Point2D 包含末端执行器的x和y坐标的结构体
 */
 
Point forward_clac(float theta1, float theta2) {
    // 计算中间角度beta，用于后续计算
    float beta = (theta2 - theta1) / 2.0;

    // 计算从基座到末端执行器的直线距离L
    // 使用余弦定理和勾股定理计算
    float L = LINK_LENGTH_1 * std::cos(static_cast<double>(beta)) + 
               sqrt(LINK_LENGTH_2 * LINK_LENGTH_2 - 
                    LINK_LENGTH_1 * LINK_LENGTH_1 * sin(static_cast<double>(beta)) * sin(static_cast<double>(beta)));

    // 声明一个Point2D结构体变量footPosition，用于存储计算得到的末端执行器位置
    Point footPosition;

    // 计算末端执行器的x坐标
    // 使用三角函数计算x坐标
    footPosition.x = L * cos(static_cast<double>(beta + theta1));

    // 计算末端执行器的y坐标
    // 使用三角函数计算y坐标
    footPosition.y = L * sin(static_cast<double>(beta + theta1));

    // 返回包含末端执行器位置的Point2D结构体
    return footPosition;

}

/**
 * @brief 实现机器人的逆运动学计算，即根据末端执行器的位置计算出关节角度。
 * 
 * @param x 末端执行器的x坐标
 * @param y 末端执行器的y坐标
 * @return JointAngles 包含两个关节角度的结构体
 */

JointAngles reverse_clac(float x, float y) {
	
    // 计算从基座到末端执行器的直线距离
    double L = sqrtf(x * x + y * y);

      // 检查目标是否可达
    if (L > LINK_LENGTH_1 + LINK_LENGTH_2 || L < fabsf(LINK_LENGTH_1 - LINK_LENGTH_2)) {
        // 如果目标位置不可达，抛出异常
        std::runtime_error("Target position is out of reach.");
    }

    // 计算从基座到末端执行器的连线与x轴正方向的夹角
    double atan_yx = atan2f(y, x);
    // 计算一个中间角度beta，用于后续计算关节角度
    // 使用余弦定理计算beta
    double beta = acos((L * L + LINK_LENGTH_1 * LINK_LENGTH_1 - LINK_LENGTH_2 * LINK_LENGTH_2) / (2 * L * LINK_LENGTH_1));
    // 声明一个JointAngles结构体变量angles，用于存储计算得到的关节角度
    JointAngles angles;
    // 计算第一个关节角度theta1
    angles.theta1 = atan_yx - beta;
    // 计算第二个关节角度theta2
    angles.theta2 = atan_yx + beta;
    // 返回包含计算得到的关节角度的JointAngles结构体
    return angles;
}
/**
 * @brief 将关节角度（弧度）转换为电机编码器值
 * 
 * @param joint_rad 关节角度（弧度）
 * @return uint16_t 电机编码器值
 */
uint32_t jointToMotorEncoder(float joint_rad) {
    // 定义常量：弧度转角度的系数
    const float RAD_TO_DEG = 180.0f / PI; 
    // 定义常量：编码器分辨率
    const float ENCODER_RESOLUTION = 8192; 

    // 转换为电机角度：将关节角度转换为电机实际旋转的角度
    float motor_angle = joint_rad * RAD_TO_DEG * 19.2f; // 19.2为电机减速比

    // 转换为编码器值：将电机角度转换为编码器的值
    uint32_t encoder_value = static_cast<uint16_t>((motor_angle / 360.0f) * ENCODER_RESOLUTION);

    // 返回计算得到的编码器值
    return encoder_value;
}




