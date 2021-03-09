#include "vector.h"
#include <math.h>
#include<stdio.h>

float dot(float ax, float ay, float az, float bx, float by, float bz){
  float dotres = 0;
  dotres += ax * bx;
  dotres += ay * by;
  dotres += az * bz;
  return dotres;
}

float norm (float x, float y, float z) {
  float acc = 0;
  acc += x*x;
  acc += y*y;
  acc += z*z;
  return sqrt(acc);
}

float vectorAngle(float ax, float ay, float az, float bx, float by, float bz){
  float dotprod = dot(ax, ay, az, bx, by, bz);
  return asin(dotprod/(norm(ax,ay,az)*norm(bx,by,bz)));
}

float vectorAngle2(float ax, float ay, float az, float bx, float by, float bz){
  float dotprod = dot(ax, ay, az, bx, by, bz);
  return acos(dotprod/(norm(ax,ay,az)*norm(bx,by,bz)));
}

float toDeg(float a){
	return a * 180 / M_PI;
}

float toRad(float a){
	return a * M_PI / 180 ;
}
  
int sign(float a){
	if (a != 0){ return a/fabs(a);}
	else { return 1;}
}

float wrapAngle(float rad){
	float angle = fmod(toDeg(rad),(float)(sign(rad)*360));
	if (angle > 180) { angle = -(360-angle); }
	else if (angle <-180) { angle = (360 + angle); }
	else {angle = angle;}
	return toRad(angle);
}

float wrapVal(float a, float b){
	if(a<-b){
		return -b;
	} else if (a>b) {
		return b;
	} else {
		return a;
	}
}
