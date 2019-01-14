//----------------------------------------------------------------------------------------------------------------------
// GRVC AUTOPILOT
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#ifndef COMMON_H_
#define COMMON_H_

#include <iostream>
#include <sstream>
#include <arpa/inet.h>
#include <MPU9250.h>
#include <LSM9DS1.h>
#include <MS5611.h>
#include <ADC_Navio2.h>
#include <RCInput_Navio2.h>
#include <RCOutput_Navio2.h>
#include <Ublox.h>
#include <navio_types.h>

#define SERVO_MIN 1100 /*mS*/
#define SERVO_MAX 2000 /*mS*/

#define RASP_IP "10.0.0.65"	  //"pi@navio2AE-1.local"
#define ODROID_IP "10.0.0.31"	  //"pi@navio2AE-1.local"
#define PC_IP "aegrvc.local" //"10.0.0.200"
#define UDP_PORT_TOTALSTATION "8000"
#define UDP_PORT_ODROID "44000"

#define BUFLEN 512 //tamaño maximo del buffer

typedef struct
{
	std::vector<double> _pos_data;
	Ublox _gps;
	shm_gps _shmmsg;
}gps_str;

typedef struct
{
	shm_status * _shmmsg;
}status_str;

typedef struct
{
	shm_px4flow _shmmsg;
}px4flow_str;
typedef struct
{
	shm_totalStation _shmmsg;
}totalStation_str;

typedef struct
{
	float _x;
	float _y;
	float _z;
	uint64_t _time;

}UDPTotalStation_msg;

typedef struct
{
	shm_lightware _shmmsg;
}sf11c_str;

typedef struct
{
	ADC_Navio2 _adc;
	shm_adc _shmmsg;
}adc_str;

typedef struct
{
	MS5611 _baro;
	shm_barometer _shmmsg;
}baro_str;

typedef struct
{
	MPU9250 _mpu9250_imu;
	shm_imu _shmmsg;
}mpu9250_imu_str;

typedef struct
{
	LSM9DS1 _lsm9ds1_imu;
	shm_imu _shmmsg;
}lsm9ds1_imu_str;


typedef struct
{
	bool _calib_rotors_required = false;
	RCInput_Navio2 _rcinput;
	shm_RCin _shmmsg_rcin;
	RCOutput_Navio2 _pwm;
	shm_RCou * _shmmsg_rcout;

}rcio_str;

typedef struct
{
	int _sectimestamp;    // timestamp en secs.
	int _nsectimestamp;    // timestamp en nsecs.
	float _x_s;        // x position
	float _y_s;        // y position
	float _z_s;        // z position
	float _quat[4];    // x, y, z, w, orientation
}UDPSVO_msg;

typedef struct
{
	shm_svo _shmmsg;
}svo_str;

/// Employ this function to use switch-case with strings
constexpr unsigned int fhash(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (fhash(str, h+1)*33) ^ str[h];
}

/// Convert ‘const char*’ to ‘short unsigned int’
inline unsigned int c_uint(const char* str)
{
	std::stringstream strValue;
	strValue << str;
	unsigned int intValue;
	strValue >> intValue;
	return intValue;
}

#endif /* COMMON_H_ */
