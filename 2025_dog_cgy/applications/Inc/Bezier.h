#ifndef _BEZIER_H
#define _BEZIER_H

#define ZU_x 0.0
#define ZU_y 320.0

#include <vector>
#include <cmath>

using namespace std;



// 点的定义
struct Point {   
    float x, y; // 点的坐标
    Point() : x(0), y(0) {}
    Point(float x, float y) : x(x), y(y) {}
};



// 步态参数结构体
 typedef struct  {
    float stepLength;  // 步长
    float stepHeight;  // 抬腿高度
    float period;      // 总周期
    float swingRatio;  // 摆动相比例
    int numSamples;    // 采样点数
}GaitParams;
 
struct QuadrupedGait {
    GaitParams params[4];  // 四条腿的步态参数
    float phaseOffsets[4]; // 每条腿的相位偏移
};

void FullTra(const QuadrupedGait& gait, Point trajectories[4][100]);

	
#endif