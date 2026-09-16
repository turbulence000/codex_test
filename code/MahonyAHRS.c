//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include "zf_common_headfile.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	200.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.2f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions
volatile float invsampleFreq = 1.0f / sampleFreq;
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	static float recipNorm;
	static float halfvx, halfvy, halfvz;
	static float halfex, halfey, halfez;
	static float qa, qb, qc;

	// �������ٶȼƲ�����Чʱ�ż��㷴����������ٶȼƹ�һ���е�NaN��
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

	    //ת��Ϊrad/s
	    gx*=0.0174532925f;
	    gy*=0.0174532925f;
	    gz*=0.0174532925f;

	    // ��һ�����ٶȼƲ���ֵ
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// ������������ʹ�ֱ�ڴ�ͨ����ʸ��
		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// ����ǹ������������������������ĳ˻�֮��
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// ������ã����㲢Ӧ�û��ַ���
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * invsampleFreq;	// ������Ki����
			integralFBy += twoKi * halfey * invsampleFreq;  // integral error scaled by Ki
			integralFBz += twoKi * halfez * invsampleFreq;
			gx += integralFBx;	// Ӧ�û��ַ���
			gy += integralFBy;  // apply integral feedback
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// ��ֹ���ֱ���
			integralFBy = 0.0f; // prevent integral windup
			integralFBz = 0.0f;
		}

		// Ӧ�ñ�������
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// ������Ԫ���仯��
	// Integrate rate of change of quaternion
	gx *= (0.5f * invsampleFreq);		// Ԥ�˳�������
	gy *= (0.5f * invsampleFreq);       // pre-multiply common factors
	gz *= (0.5f * invsampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// ��һ����Ԫ��
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

//void quaternion_to_euler(float q[4], float euler[3])
	static float r11,r12,r21,r31,r32;
    // ������ת�����Ԫ��
    r11 = 2.0f * (q0*q1 + q2*q3);
    r12 = 1.0f - 2.0f * (q1*q1 + q2*q2);
    r21 = 2.0f * (q0*q2 - q3*q1);
    r31 = 2.0f * (q0*q3 + q1*q2);
    r32 = 1.0f - 2.0f * (q2*q2 + q3*q3);

    // ����ŷ����(����)
    Yaw_a_Pi = atan2f(r31, r32);
    Pitch_a_Pi = -asinf(r21);
    Roll_a_Pi = atan2f(r11, r12);

}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
