#ifndef USER_TYPES_H_
#define USER_TYPES_H_

#include "stdint.h"

#define M_PI_F 3.1415926

typedef struct gyro_f {
	float x;
	float y;
	float z;
} GYRO_F_t;

typedef struct gyro_i {
	int16_t x;
	int16_t y;
	int16_t z;
} GYRO_I_t;

typedef struct accel_f {
    float x;
    float y;
    float z;
} ACCEL_F_t;

typedef struct accel_i {
	int16_t x;
	int16_t y;
	int16_t z;
} ACCEL_I_t;

typedef struct quat {
	float q0;
	float q1;
	float q2;
	float q3;
} QUAT_t;

typedef struct angles {
    float pitch;
    float roll;
} ANGLES_t;

#endif
