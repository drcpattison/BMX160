//=============================================================================================
// Mahony.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef __Mahony_BMX160_h__
#define __Mahony_BMX160_h__
#include <stdint.h>
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony_BMX160 {
private:
	double delta_t = 0; // Used to control display output rate
	uint32_t now = 0;        // used to calculate integration interval
	uint32_t last_update = 0; // used to calculate integration interval
	double delta_t1 = 0; // Used to control display output rate
	uint32_t now1 = 0;        // used to calculate integration interval
	uint32_t last_update1 = 0; // used to calculate integration interval
	float twoKp;		// 2 * proportional gain (Kp)
	float twoKi;		// 2 * integral gain (Ki)
	float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
	float roll, pitch, yaw;
	char anglesComputed;
	static float invSqrt(float x);
	void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony_BMX160();
	void begin() { }
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, uint32_t now);
	void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, uint32_t now);
	float getRoll() {
		if (!anglesComputed) computeAngles();
		return roll * 57.29578f;
	}
	float getPitch() {
		if (!anglesComputed) computeAngles();
		return pitch * 57.29578f;
	}
	float getYaw() {
		if (!anglesComputed) computeAngles();
		return yaw * 57.29578f + 180.0f;
	}
	float getRollRadians() {
		if (!anglesComputed) computeAngles();
		return roll;
	}
	float getPitchRadians() {
		if (!anglesComputed) computeAngles();
		return pitch;
	}
	float getYawRadians() {
		if (!anglesComputed) computeAngles();
		return yaw;
	}
	void getQuaternion(float *w, float *x, float *y, float *z) {
       *w = q0;
       *x = q1;
       *y = q2;
       *z = q3;
   }
};

#endif
