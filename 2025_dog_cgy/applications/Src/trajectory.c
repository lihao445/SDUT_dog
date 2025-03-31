#include "trajectory.h"
#include <math.h>

typedef struct {
    float x;
    float y;
} TrajectoryPoint;

TrajectoryPoint lut[LUT_SIZE]; // 轨迹查找表

/**
 * @brief 初始化轨迹查找表
 * 此函数用于预计算一个周期内的所有轨迹点，并将其存储在查找表中。
 * 轨迹分为摆动相（空中运动）和支撑相（地面接触）两个阶段。
 * 
 * @param 无
 * @return 无
 */
 
 float swing_phase_ratio = 0.5f;

void InitTrajectoryLUT() 
{
    // 遍历整个查找表，预计算所有轨迹点
    for (int i = 0; i < LUT_SIZE; i++) {
        // 时间映射：将索引转换为[0, CYCLE_TIME]范围内的实际时间
        float t = (float)i / (LUT_SIZE - 1) * CYCLE_TIME;
        float t_norm = fmod(t, CYCLE_TIME) / CYCLE_TIME; // 归一化到[0,1]区间

        // 摆动相（足端空中移动阶段）
        if (t_norm <= swing_phase_ratio) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
            float t_swing = t_norm / swing_phase_ratio;
            
            // --- X轴轨迹计算（五次多项式） ---
            float t2 = t_swing * t_swing;
            float t3 = t2 * t_swing;
            float t4 = t3 * t_swing;
            float t5 = t4 * t_swing;
            lut[i].x = -STEP_LENGTH/2 + STEP_LENGTH * (10*t3*(1-t_swing)*(1-t_swing) + 5*t4*(1-t_swing) + t5);

            // Y轴轨迹分两阶段处理（抬腿+放腿）
            if (t_norm <= swing_phase_ratio * 0.5f) { 
                // 抬腿阶段（占摆动相前半段）
                float t_local = t_norm / (swing_phase_ratio * 0.5f);
                lut[i].y = BODY_HEIGHT + LIFT_HEIGHT * 
                    (10*powf(t_local,3)*powf(1-t_local,2) 
                    + 5*powf(t_local,4)*(1-t_local) 
                    + powf(t_local,5));
            } else {  // 放腿阶段 
                float t_local = (t_norm - swing_phase_ratio*0.5f) / (swing_phase_ratio*0.5f);
                lut[i].y = BODY_HEIGHT + LIFT_HEIGHT * 
                    (10*powf(1-t_local,3)*powf(t_local,2) 
                    + 5*powf(1-t_local,4)*t_local 
                    + powf(1-t_local,5));
            }
        } 
        // 支撑相（足端地面接触阶段）
        else {
            // --- 支撑相阶段处理 ---
            float t_support = (t_norm - swing_phase_ratio) / (1 - swing_phase_ratio);
            // X轴从STEP_LENGTH/2线性移动到-STEP_LENGTH/2
            lut[i].x = STEP_LENGTH/2 - STEP_LENGTH * t_support;
            // Y轴保持地面高度
            lut[i].y = BODY_HEIGHT;
        }
    }
}

// 获取轨迹点（查表法）
/**
 * @brief 根据给定时间获取轨迹点的坐标
 * 该函数通过输入的时间参数，从预计算好的轨迹查找表中获取对应的轨迹点坐标。
 * 
 * @param t 输入的时间，用于确定在查找表中的位置
 * @param x 指向存储X坐标的变量的指针，用于返回轨迹点的X坐标
 * @param y 指向存储Y坐标的变量的指针，用于返回轨迹点的Y坐标
 * @return 无
 */
void GetTrajectoryPoint(float t, float *x, float *y) 
{
    // 优化后的索引计算（直接映射到LUT_SIZE）
    int index = (int)(t / CYCLE_TIME * LUT_SIZE);//按比例映射到查找表大小
    
    // 简化边界检查（因调用方已做周期处理）
    if (index >= LUT_SIZE) index = LUT_SIZE - 1;
    
    // 查表操作保持不变
    *x = lut[index].x;
    *y = lut[index].y;
}