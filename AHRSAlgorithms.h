#ifndef _AHRSALOGRITHMS_H_
#define _AHRSALOGRITHMS_H_

#include <cmath>
#define PI 3.14159265358979323846f
#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
const float * getQ();

#endif // _AHRSALOGRITHMS_H_
