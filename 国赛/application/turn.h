#ifndef __TURN_H
#define __TURN_H

#include "sys.h"
#include "imu_task.h"

struct Angle {
	float AngleT;
	float AngleG;
};

extern volatile struct Angle angle;

float need2turn(float nowangle,float targetangle);
void mpuZreset(float sensorangle ,float referangle);
float getAngleZ(void);
	
uint8_t Turn_Angle(float Angle);
void Turn_Angle_Relative(float Angle1);
uint8_t runWithAngle(float angle_want,float speed);
uint8_t Drift(float speed,float Angle);
uint8_t Small_Turn_Angle(float Angle);
void AdCircle(float speed, float radius);
static inline float get_pitch(void)
{
	return imu.pitch;
}


#endif

