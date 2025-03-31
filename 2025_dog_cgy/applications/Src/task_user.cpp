#include "task_user.h"
#include "math.h"
 
//extern "C"
void walk_m(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float t = 0.0f;
    
    // 新增：相位偏移数组 [左前, 右前, 右后, 左后]
    const float phase_offsets[4] = {0.0f, 0.5f*CYCLE_TIME, 0.0f, 0.5f*CYCLE_TIME};
    
    InitTrajectoryLUT();

    while (1) {
           // 计算相位调整后的时间
           float t_phased[4]={fmodf(t + phase_offsets[0], CYCLE_TIME),
                              fmodf(t + phase_offsets[1], CYCLE_TIME),
                              fmodf(t + phase_offsets[2], CYCLE_TIME),
                              fmodf(t + phase_offsets[3], CYCLE_TIME)};
           
           // 获取原始轨迹点
           float x[4], y[4];
           for(int i = 0; i < 4; i++) {
               GetTrajectoryPoint(t_phased[i], &x[i], &y[i]);
           }

           // 计算逆运动学
           float theta1[4], theta2[4];
           for(int i = 0; i < 4; i++) {
               //InverseKinematics(x[i], y[i], &theta1[i], &theta2[i]);
           }
           
           // 控制电机
          //ControlLegMotors(&theta1[0],&theta2[0]);
        t += 0.001f;
        if(t >= CYCLE_TIME) t -= CYCLE_TIME;  

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }










 


































}