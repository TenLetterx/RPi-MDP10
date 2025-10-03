#ifndef INC_DIST_H_
#define INC_DIST_H_

#include "kalman.h"

#define DIST_S_ACCEL 9.80665e-6f
#define DIST_S_MOTOR 0.75f

//accurate range of IR sensor.
#define DIST_IR_MIN 6.0f  //Minimum
#define DIST_IR_MAX 70.0f //Based on documentation of IR Datasheet
#define DIST_IR_OFFSET 4.5f //distance from front of vehicle for bias

typedef struct {
	float dist;
	float v;
	float s_v;
} DistState;

void dist_track_init();
void dist_reset(float v);
float dist_get_cm(float msElapsed, float accel, float motorDist);

#endif /* INC_DIST_H_ */
