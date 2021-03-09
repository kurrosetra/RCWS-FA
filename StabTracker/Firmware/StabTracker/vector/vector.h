#ifndef VECTOR_LIB
#define VECTOR_LIB
#include<math.h>

float norm (float x, float y, float z);
float dot(float, float, float, float, float, float);
float vectorAngle(float, float, float, float, float, float);
float vectorAngle2(float, float, float, float, float, float);
float toDeg(float a);
float toRad(float a);
int sign(float a);
float wrapAngle(float rad);
float wrapVal(float a, float b);

#endif
