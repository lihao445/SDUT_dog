#ifndef _MOTION_CLAC_H
#define _MOTION_CLAC_H

#define PI 3.14159265358979323846
#define LINK_LENGTH_1 150.0f // Length of the first link
#define LINK_LENGTH_2 300.0f // Length of the second link

#include "Bezier.h"



struct JointAngles {
    float theta1, theta2;
};


Point forward_clac(float theta1, float theta2);
JointAngles reverse_clac(float x, float y);
uint32_t jointToMotorEncoder(float joint_rad);

#endif