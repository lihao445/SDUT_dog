#ifndef _BEZIER_H
#define _BEZIER_H

#define ZU_x 0.0
#define ZU_y 320.0

#include <vector>
#include <cmath>

using namespace std;



// ��Ķ���
struct Point {   
    float x, y; // �������
    Point() : x(0), y(0) {}
    Point(float x, float y) : x(x), y(y) {}
};



// ��̬�����ṹ��
 typedef struct  {
    float stepLength;  // ����
    float stepHeight;  // ̧�ȸ߶�
    float period;      // ������
    float swingRatio;  // �ڶ������
    int numSamples;    // ��������
}GaitParams;
 
struct QuadrupedGait {
    GaitParams params[4];  // �����ȵĲ�̬����
    float phaseOffsets[4]; // ÿ���ȵ���λƫ��
};

void FullTra(const QuadrupedGait& gait, Point trajectories[4][100]);

	
#endif