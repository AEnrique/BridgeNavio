/*
 * navio_types.h
 *
 *  Created on: 1/12/2017
 *      Author: GRVC
 */

#ifndef NAVIO_TYPES_H_
#define NAVIO_TYPES_H_

#include <stdint.h>

typedef struct
{
    float _temperature;
    float _ax;
    float _ay;
    float _az;
    float _gx;
    float _gy;
    float _gz;
    float _mx;
    float _my;
    float _mz;
}shm_imu;

typedef struct
{
    float _q0, _q1, _q2, _q3;
}shm_ahrs;

typedef struct
{
    float _z;
}shm_lightware;

typedef struct
{
    float _mx, _my, _mz;
}shm_magnetometer;

typedef struct
{
	float _temp; // Calculated temperature
    float _pres; // Calculated pressure
}shm_barometer;

typedef struct
{
	float _adc0;
	float _adc1;
	float _adc2;
	float _adc3;
	float _adc4;
	float _adc5;
}shm_adc;


typedef struct
{
	unsigned short _channel1;
	unsigned short _channel2;
	unsigned short _channel3;
	unsigned short _channel4;
	unsigned short _channel5;
	unsigned short _channel6;
	unsigned short _channel7;
	unsigned short _channel8;
	unsigned short _channel9;
	unsigned short _channel10;
	unsigned short _channel11;
	unsigned short _channel12;
	unsigned short _channel13;
	unsigned short _channel14;
}shm_RCin;

typedef struct
{
	float _channel1;
	float _channel2;
	float _channel3;
	float _channel4;
	float _channel5;
	float _channel6;
	float _channel7;
	float _channel8;
	float _channel9;
	float _channel10;
	float _channel11;
	float _channel12;
	float _channel13;
	float _channel14;
}shm_RCou;

typedef struct
{
	float _x;
	float _y;
	float _z;
	double _time;

}shm_totalStation;

typedef struct
{
  unsigned short _status;
  double _time;
  double _lon;
  double _lat;
  double _height_ellipsoid;
  double _height_sea;
  double _hor_accur;
  double _ver_accur;
}shm_gps;

/*typedef struct
{
	int8_t _sensor_id;
	int16_t _quality;
	float _flow_comp_m_x;
	float _flow_comp_m_y;
	float _ground_distance;
	int16_t _flow_x;
	int16_t _flow_y;
} shm_px4flow;*/

typedef struct
{
	uint16_t _frame_count;// counts created I2C frames [#frames]
	int16_t _pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
	int16_t _pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
	int16_t _flow_comp_m_x;// x velocity*1000 [meters/sec]
	int16_t _flow_comp_m_y;// y velocity*1000 [meters/sec]
	int16_t _qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
	int16_t _gyro_x_rate; // latest gyro x rate [rad/sec]
	int16_t _gyro_y_rate; // latest gyro y rate [rad/sec]
	int16_t _gyro_z_rate; // latest gyro z rate [rad/sec]
	uint8_t _gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
	uint8_t _sonar_timestamp;// time since last sonar update [milliseconds]
	int16_t _ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance

}shm_px4flow;

#endif /* NAVIO_TYPES_H_ */
