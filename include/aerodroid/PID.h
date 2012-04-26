#ifndef __PID_H__
#define __PID_H__

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float IntegratedError;
	float LastError;
  float IMax;
} PID_TYPE;

float doPID(float target, float current, PID_TYPE* PID);

#endif
