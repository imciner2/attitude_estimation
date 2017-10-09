//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 09/10/2017   I   McInerney   Redid quaternion access
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include "attitudeTypes.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)

//---------------------------------------------------------------------------------------------------
// Function declarations
#if defined (__cplusplus)
extern "C" {
#endif

void MahonyAHRSgetQuaternions(QUAT_t *quat);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#if defined (__cplusplus)
}
#endif

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
