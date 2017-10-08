#ifndef ATTITUDE_CONVERSION_H_
#define ATTITUDE_CONVERSION_H_

#include "attitudeTypes.h"

/*
 * Convert the vector from the body frame to the inertial frame
 *
 * @param body The body frame vector
 * @param inertial ACCEL_F_t to hold the new inertial vector
 * @param quat The quaternion representation of the attitude
 */
void bodyToInertial(ACCEL_F_t body, ACCEL_F_t *inertial, QUAT_t quat);

/**
 * Compute the gravitational vector from the quaternion estimate
 *
 * @param vec Pointer to the ACCEL_F_t to store the vector in
 * @param quat The QUAT_t the quaternion orientation is stored in
 *
 */
void getGravVector(ACCEL_F_t *vec, QUAT_t quat);

/**
 * Compute the Euler angles from the quaternion estimate
 *
 * @param angles Pointer to the ANGLES_t to store the computed angles in
 * @param quat The QUAT_t the quaternion orientation si stored in
 */
void getEulerAngles(ANGLES_t *angles, QUAT_t quat);

#endif
