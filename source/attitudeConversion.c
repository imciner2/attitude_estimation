#include "MahonyAHRS.h"
#include "attitudeTypes.h"
#include "attitudeConversion.h"

#include <math.h>

/*
 * Convert the vector from the body frame to the inertial frame
 *
 * @param body The body frame vector
 * @param inertial ACCEL_F_t to hold the new inertial vector
 * @param quat The quaternion representation of the attitude
 */
void bodyToInertial(ACCEL_F_t body, ACCEL_F_t *inertial, QUAT_t quat) {
	// Create some temporary variables
	float q0 = quat.q0;
	float q1 = quat.q1;
	float q2 = quat.q2;
	float q3 = quat.q3;

	// Compute the conversion matrix
	/*
	 * Conversion Matrix:
	 * | l1  m1  n1 |
	 * | l2  m2  n2 |
	 * | l3  m3  n3 |
	 */
	float l1 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	float l2 = 2 * (q1 * q2 + q0 * q3);
	float l3 = 2 * (q1 * q3 - q0 * q2);

	float m1 = 2 * (q1 * q2 - q0 * q3);
	float m2 = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
	float m3 = 2 * (q2 *q3 + q0 * q1);

	float n1 = 2 * (q1 * q3 + q0 * q2);
	float n2 = 2 * (q2 * q3 - q0 * q1);
	float n3 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	// Compute the resultant vectors
	inertial->x = (l1 * body.x) + (m1 * body.y) + (n1 * body.z);
	inertial->y = (l2 * body.x) + (m2 * body.y) + (n2 * body.z);
	inertial->z = (l3 * body.x) + (m3 * body.y) + (n3 * body.z);
}

/**
 * Compute the gravitational vector from the quaternion estimate
 *
 * @param vec Pointer to the ACCEL_F_t to store the vector in
 * @param quat The QUAT_t the quaternion orientation is stored in
 *
 */
void getGravVector(ACCEL_F_t *vec, QUAT_t quat) {
	vec->x = 2 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
	vec->y = 2 * (quat.q0 * quat.q1 + quat.q2 * quat.q3);
	vec->z = quat.q0 * quat.q0 - quat.q1 * quat.q1 - quat.q2 * quat.q2 + quat.q3 * quat.q3;
}

/**
 * Compute the Euler angles from the quaternion estimate
 *
 * @param angles Pointer to the ANGLES_t to store the computed angles in
 * @param quat The QUAT_t the quaternion orientation si stored in
 */
void getEulerAngles(ANGLES_t *angles, QUAT_t quat) {
	ACCEL_F_t grav;
	getGravVector(&grav, quat);

	angles->roll = atan2f(grav.y, grav.z) * 180/M_PI_F;
	angles->pitch = asinf(-grav.x) * 180/M_PI_F;
}
