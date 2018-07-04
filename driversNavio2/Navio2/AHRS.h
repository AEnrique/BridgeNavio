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
//

#ifndef AHRS_H_
#define AHRS_H_

#include <cmath>
#include <stdio.h>
#include <memory>
#include "InertialSensor.h"

class AHRS{
private:
	float q0, q1, q2, q3;
	float gyroOffset[3];
	float twoKi;
	float twoKp;
	float integralFBx, integralFBy, integralFBz;
    //std::unique_ptr <InertialSensor> sensor;
public:
    //AHRS( std::unique_ptr <InertialSensor> imu);
	AHRS();
	~AHRS();

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float dt);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float dt);
    //void setGyroOffset();
    void getEuler(float* roll, float* pitch, float* yaw);

    float invSqrt(float x);
    float getW();
    float getX();
    float getY();
    float getZ();
};


#endif /* AHRS_H_ */
