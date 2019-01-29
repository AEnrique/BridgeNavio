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

#ifndef NAVIO_TYPES_H_
#define NAVIO_TYPES_H_

#include <stdint.h>
typedef struct
{
    uint8_t _heartbeat;
    double _time;
    uint8_t _isStarted;
    double _vehicle_attitude_desired[6];
    double _vehicle_attitude[6];
    double _vehicle_position_desired[6];
    double _vehicle_position[6];
    double _pwm_commanded[14];
}shm_status;

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
	float _x;        // x position
	float _y;        // y position
	float _z;        // z position
	float _qx;    	   // qx orientation
	float _qy;         // qy orientation
	float _qz;         // qz orientation
	float _qw;         // qw orientation

}shm_svo;


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


typedef struct
{
	float     _ground_distance; //  # distance to ground in meters
	int16_t   _flow_x; //           # x-component of optical flow in pixels
	int16_t   _flow_y; //           # y-component of optical flow in pixels
	float     _velocity_x; //       # x-component of scaled optical flow in m/s
	float     _velocity_y; //       # y-component of scaled optical flow in m/s
	uint8_t   _quality; //          # quality of optical flow estimate

}shm_px4flow;

typedef struct
{
	double _x;
	double _y;
	double _z;

	double _qx;
	double _qy;
	double _qz;
	double _qw;

}shm_vicon;

typedef struct{

	float _rollspeed;	/**< Roll angular speed (rad/s, Tait-Bryan, NED)		*/
	float _pitchspeed;	/**< Pitch angular speed (rad/s, Tait-Bryan, NED)		*/
	float _yawspeed;		/**< Yaw angular speed (rad/s, Tait-Bryan, NED)			*/
	float _q[4];		/**< Quaternion (NED)						*/

	uint8_t _check;
} shm_attitudePX4EKF2;

typedef struct{
	//uint64_t timestamp;		/**< Time of this estimate, in microseconds since system start		*/
	//uint64_t time_gps_usec;		/**< GPS timestamp in microseconds					   */
	double _lat;			/**< Latitude in degrees							 	   */
	double _lon;			/**< Longitude in degrees							 	   */
	float _alt;			/**< Altitude AMSL in meters						 	   */
	float _vel_n; 			/**< Ground north velocity, m/s				 			   */
	float _vel_e;			/**< Ground east velocity, m/s							   */
	float _vel_d;			/**< Ground downside velocity, m/s						   */
	float _yaw; 			/**< Yaw in radians -PI..+PI.							   */
	float _eph;			/**< Standard deviation of position estimate horizontally */
	float _epv; /**< Standard deviation of position vertically */
	uint8_t _check;
} shm_globalPositionPX4EKF2;

typedef struct{
	//bool _xy_valid;			/**< true if x and y are valid */
	//bool _z_valid;			/**< true if z is valid */
	//bool _v_xy_valid;		/**< true if vy and vy are valid */
	//bool _v_z_valid;			/**< true if vz is valid */
		/* Position in local NED frame */
	float _x;				/**< X position in meters in NED earth-fixed frame */
	float _y;				/**< X position in meters in NED earth-fixed frame */
	float _z;				/**< Z position in meters in NED earth-fixed frame (negative altitude) */
		/* Velocity in NED frame */
	float _vx; 				/**< Ground X Speed (Latitude), m/s in NED */
	float _vy;				/**< Ground Y Speed (Longitude), m/s in NED */
	float _vz;				/**< Ground Z Speed (Altitude), m/s	in NED */
		/* Heading */
	float _yaw;
// Acceleration in NED frame
	float _ax;       // North velocity derivative in NED earth-fixed frame, (metres/sec^2)
	float _ay;       // East velocity derivative in NED earth-fixed frame, (metres/sec^2)
	float _az;       // Down velocity derivative in NED earth-fixed frame, (metres/sec^2)
	uint8_t _check;
} shm_localPositionPX4EKF2;



#endif /* NAVIO_TYPES_H_ */
