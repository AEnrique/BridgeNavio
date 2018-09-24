/*
 * driversNavio2.h
 *
 *  Created on: 18/12/2017
 *      Author: GRVC
 */

#ifndef DRIVERSNAVIO2_H_
#define DRIVERSNAVIO2_H_


#include <PWM.h>
#include <RCOutput_Navio2.h>
#include <unistd.h>
#include <memory>
#include <MPU9250.h>
#include <LSM9DS1.h>
#include <MS5611.h>
#include <ADC_Navio2.h>
#include <RCInput_Navio2.h>
#include <Util.h>
#include <pthread.h>
#include <sched.h>
#include <AHRS.h>
#include <Ublox.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h> //printf
#include <string.h> //memset
#include <stdlib.h> //exit(0);
#include <arpa/inet.h>
#include <sys/socket.h>
#include "I2Cdev.h"

#include <SerialComm.h>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/array.hpp>

#define BUFLEN 512  //Max length of buffer
#define UDPPORT 44000   //The port on which to listen for incoming data

#define CONNECTED_RC_SYSFS_PATH "/sys/kernel/rcio/connected"

#define READ_FAILED -1

#define SERVO_MIN 1100 /*mS*/
#define SERVO_MAX 2000 /*mS*/

#define PWM_OUTPUT1 0
#define PWM_OUTPUT2 1
#define PWM_OUTPUT3 2
#define PWM_OUTPUT4 3
#define PWM_OUTPUT5 4
#define PWM_OUTPUT6 5
#define PWM_OUTPUT7 6
#define PWM_OUTPUT8 7
#define PWM_OUTPUT9 8
#define PWM_OUTPUT10 9
#define PWM_OUTPUT11 10
#define PWM_OUTPUT12 11
#define PWM_OUTPUT13 12
#define PWM_OUTPUT14 13

uint32_t read32(uint8_t load[], int index0);
uint16_t read16(uint8_t load[], int index0);
uint8_t read8(uint8_t load[], int index0);
double ConvertFrom64toDouble(uint8_t load[], int index0);
typedef struct
{
    uint16_t frame_count;// counts created I2C frames [#frames]
    int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
    int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
    int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
    int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
    int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
    int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
    int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
    int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
    uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
    uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance
} i2c_frame;
typedef struct i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000]
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
} __attribute__((packed))i2c_integral_frame;

typedef struct
{
	//SerialComm _comm;
	shm_px4flow _shmmsg;
}px4flow_str;

typedef struct
{
	shm_lightware _shmmsg_lightware;
	shm_px4flow _shmmsg_px4flow;
}i2cSplitter_str;

typedef struct
{
	float _x;
	float _y;
	float _z;
	uint64_t _time;

}UDPTotalStation_msg;
typedef struct
{
	std::vector<double> _pos_data;
	Ublox _gps;
	shm_gps _shmmsg;
}gps_str;

typedef struct
{
	sockaddr_in _si_me, _si_other;
	shm_totalStation _shmmsg;
}totalStation_str;

typedef struct
{
	uint8_t _sensor;
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
	bool _ahrs_required = false;
	shm_imu _shmmsg;
	shm_ahrs _quaternion;
}mpu9250_imu_str;

typedef struct
{
	LSM9DS1 _lsm9ds1_imu;
	shm_imu _shmmsg;
}lsm9ds1_imu_str;

typedef struct
{
	AHRS *_ahrs;
	shm_ahrs _shmmsg;
}ahrs_str;

typedef struct
{
	bool _calib_rotors_required = false;
	int16_t _numRotors;
	RCInput_Navio2 _rcinput;
	shm_RCin _shmmsg_rcin;
	RCOutput_Navio2 _pwm;
	shm_RCou * _shmmsg_rcout;
}rcio_str;

void * sendRCIOData(void * rcio)
{

	rcio_str* rc = (rcio_str*)rcio;
	boost::asio::io_service io;

	float pwm_out[rc->_numRotors];
	int i;
	for(i = 0;i < rc->_numRotors; i++)
	{
		rc->_pwm.initialize(i);
		rc->_pwm.set_frequency(i, 400);
		rc->_pwm.enable(i);
	}

	if (rc->_calib_rotors_required){
		printf("Calibration of ESC in process...\n");
		for(i = 0;i < rc->_numRotors; i++)
		{
			rc->_pwm.set_duty_cycle(i, SERVO_MAX);
			sleep(4);
			rc->_pwm.set_duty_cycle(i, SERVO_MIN);
		}
		printf("End of calibration\n");
	}
	sleep(1);

    while (true) {
    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(2500));
    	rc->_shmmsg_rcin._channel1 = rc->_rcinput.read(0);
    	rc->_shmmsg_rcin._channel2 = rc->_rcinput.read(1);
    	rc->_shmmsg_rcin._channel3 = rc->_rcinput.read(2);
    	rc->_shmmsg_rcin._channel4 = rc->_rcinput.read(3);
    	rc->_shmmsg_rcin._channel5 = rc->_rcinput.read(4);
    	rc->_shmmsg_rcin._channel6 = rc->_rcinput.read(5);
    	rc->_shmmsg_rcin._channel7 = rc->_rcinput.read(6);
    	rc->_shmmsg_rcin._channel8 = rc->_rcinput.read(7);
    	rc->_shmmsg_rcin._channel9 = rc->_rcinput.read(8);
    	rc->_shmmsg_rcin._channel10 = rc->_rcinput.read(9);
    	rc->_shmmsg_rcin._channel11 = rc->_rcinput.read(10);
    	rc->_shmmsg_rcin._channel12 = rc->_rcinput.read(11);
    	rc->_shmmsg_rcin._channel13 = rc->_rcinput.read(12);
    	rc->_shmmsg_rcin._channel14 = rc->_rcinput.read(13);//~0.000100s


    	switch (rc->_numRotors){
    		case 4:
    	    	pwm_out[0] = rc->_shmmsg_rcout->_channel1;
    	    	pwm_out[1] = rc->_shmmsg_rcout->_channel2;
    	    	pwm_out[2] = rc->_shmmsg_rcout->_channel3;
    	    	pwm_out[3] = rc->_shmmsg_rcout->_channel4;
    			break;
    		case 6:
    	    	pwm_out[0] = rc->_shmmsg_rcout->_channel1;
    	    	pwm_out[1] = rc->_shmmsg_rcout->_channel2;
    	    	pwm_out[2] = rc->_shmmsg_rcout->_channel3;
    	    	pwm_out[3] = rc->_shmmsg_rcout->_channel4;
    	    	pwm_out[4] = rc->_shmmsg_rcout->_channel5;
    	    	pwm_out[5] = rc->_shmmsg_rcout->_channel6;
    	    	break;
    		case 8:
    	    	pwm_out[0] = rc->_shmmsg_rcout->_channel1;
    	    	pwm_out[1] = rc->_shmmsg_rcout->_channel2;
    	    	pwm_out[2] = rc->_shmmsg_rcout->_channel3;
    	    	pwm_out[3] = rc->_shmmsg_rcout->_channel4;
    	    	pwm_out[4] = rc->_shmmsg_rcout->_channel5;
    	    	pwm_out[5] = rc->_shmmsg_rcout->_channel6;
    	    	pwm_out[6] = rc->_shmmsg_rcout->_channel7;
    	    	pwm_out[7] = rc->_shmmsg_rcout->_channel8;
    	    	break;
    		default:
    	    	pwm_out[0] = rc->_shmmsg_rcout->_channel1;
    	    	pwm_out[1] = rc->_shmmsg_rcout->_channel2;
    	    	pwm_out[2] = rc->_shmmsg_rcout->_channel3;
    	    	pwm_out[3] = rc->_shmmsg_rcout->_channel4;
    	}


    	for(i = 0;i < rc->_numRotors; i++)
    	{
    		if (pwm_out[i] < SERVO_MIN){
    			rc->_pwm.set_duty_cycle(i, SERVO_MIN);
    		}else{
    			if (pwm_out[i] > SERVO_MAX){
    				rc->_pwm.set_duty_cycle(i, SERVO_MIN);
    			}else{
    				rc->_pwm.set_duty_cycle(i, pwm_out[i]);
    			}
    		}

    	}
    	t.wait();
        //usleep(2000);
    }
    pthread_exit(NULL);
}

void * acquireSF11CData(void * sf11c)
{
	sf11c_str* sf11c_data = (sf11c_str*)sf11c;
	boost::asio::io_service io;
	int fd;
	unsigned char buf[2];
	uint16_t measZ;
	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)//with Grove cape i2c-2 is i2c-1 and vice.
	{
    // Open port for reading and writing
		fprintf(stderr, "Failed to open i2c bus\n");
		exit(1);
	}

	while (true){
		boost::asio::deadline_timer t(io, boost::posix_time::microseconds(50000));
		if (ioctl(fd, I2C_SLAVE, 0x66) < 0) {
		    printf("Failed to acquire bus access and/or talk to slave.\n");
		    /* ERROR HANDLING; you can check errno to see what went wrong */
		    exit(1);
		}else{
			if (read(fd, buf, 2) != 2) {
				printf("Unable to read from SF11C\r");
				fflush(stdout);
			} else {
				measZ = ((uint16_t)buf[0] << 8) | buf[1];
				sf11c_data->_shmmsg._z = float(measZ)*1e-2f;

			}
		}
		t.wait();

	}
	pthread_exit(NULL);
}

void * acquireADCData(void * adc)
{
	adc_str* adc_data = (adc_str*)adc;
	boost::asio::io_service io;
    float results[adc_data->_adc.get_channel_count()] = {0.0f};
    while (true) {
    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(500000));
        for (int i = 0; i < adc_data->_adc.get_channel_count(); i++)
        {
            results[i] = adc_data->_adc.read(i);

            //if (results[i] == -1)
            //    return -1;
            //printf("A%d: %.4fV ", i, results[i] / 1000);
        }
        //printf("\r");
        //fflush(stdout);
        adc_data->_shmmsg._adc0 = results[0];
        adc_data->_shmmsg._adc1 = results[1];
        adc_data->_shmmsg._adc2 = results[2];
        adc_data->_shmmsg._adc3 = results[3];
        adc_data->_shmmsg._adc4 = results[4];
        adc_data->_shmmsg._adc5 = results[5];
        //usleep(500000);
        t.wait();
    }

    pthread_exit(NULL);
}

void * acquireLSM9DS1Data(void * imu)
{
	lsm9ds1_imu_str *lsm9ds1_imu = (lsm9ds1_imu_str*)imu;
	boost::asio::io_service io;

    while (true) {
    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(2500));
		lsm9ds1_imu->_lsm9ds1_imu.update();
		lsm9ds1_imu->_lsm9ds1_imu.read_accelerometer(&lsm9ds1_imu->_shmmsg._ax, &lsm9ds1_imu->_shmmsg._ay, &lsm9ds1_imu->_shmmsg._az);
		lsm9ds1_imu->_lsm9ds1_imu.read_gyroscope(&lsm9ds1_imu->_shmmsg._gx, &lsm9ds1_imu->_shmmsg._gy, &lsm9ds1_imu->_shmmsg._gz);
		lsm9ds1_imu->_lsm9ds1_imu.read_magnetometer(&lsm9ds1_imu->_shmmsg._mx, &lsm9ds1_imu->_shmmsg._my, &lsm9ds1_imu->_shmmsg._mz);
		lsm9ds1_imu->_shmmsg._temperature = lsm9ds1_imu->_lsm9ds1_imu.read_temperature();//~0.000750s
        //usleep(1000);
		t.wait();
    }

    pthread_exit(NULL);
}

void * acquireMPU9250Data(void * imu)
{
	boost::asio::io_service io;
	mpu9250_imu_str *mpu9250_imu = (mpu9250_imu_str*)imu;

	AHRS *_ahrs = new AHRS();
	float dt;
	struct timeval tv;

   	static unsigned long previoustime, currenttime;
    while (true) {
    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(2500));
    	mpu9250_imu->_mpu9250_imu.update();
    	mpu9250_imu->_mpu9250_imu.read_accelerometer(&mpu9250_imu->_shmmsg._ax, &mpu9250_imu->_shmmsg._ay, &mpu9250_imu->_shmmsg._az);
    	mpu9250_imu->_mpu9250_imu.read_gyroscope(&mpu9250_imu->_shmmsg._gx, &mpu9250_imu->_shmmsg._gy, &mpu9250_imu->_shmmsg._gz);
    	mpu9250_imu->_mpu9250_imu.read_magnetometer(&mpu9250_imu->_shmmsg._mx, &mpu9250_imu->_shmmsg._my, &mpu9250_imu->_shmmsg._mz);
    	mpu9250_imu->_shmmsg._temperature = mpu9250_imu->_mpu9250_imu.read_temperature();//~0.000450s
    	if (mpu9250_imu->_ahrs_required){
    		gettimeofday(&tv,NULL);
    		previoustime = currenttime;
    		currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
    		dt = (currenttime - previoustime) / 1000000.0;
    		if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
    			gettimeofday(&tv,NULL);
    			currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
    			dt = (currenttime - previoustime) / 1000000.0;


			//gettimeofday(&ts1,NULL);
			_ahrs->update(mpu9250_imu->_shmmsg._gx, mpu9250_imu->_shmmsg._gy, mpu9250_imu->_shmmsg._gz, mpu9250_imu->_shmmsg._ax, mpu9250_imu->_shmmsg._ay, mpu9250_imu->_shmmsg._az, mpu9250_imu->_shmmsg._mx, mpu9250_imu->_shmmsg._my, mpu9250_imu->_shmmsg._mz, dt);
			mpu9250_imu->_quaternion._q0 = _ahrs->getW();
			mpu9250_imu->_quaternion._q1 = _ahrs->getX();
			mpu9250_imu->_quaternion._q2 = _ahrs->getY();
			mpu9250_imu->_quaternion._q3 = _ahrs->getZ();
    	}else{
			mpu9250_imu->_quaternion._q0 = 1;
			mpu9250_imu->_quaternion._q1 = 0;
			mpu9250_imu->_quaternion._q2 = 0;
			mpu9250_imu->_quaternion._q3 = 0;
    	}

        //usleep(1000);
        t.wait();
    }

    pthread_exit(NULL);
}


void * acquireBarometerData(void * barom)
{
	baro_str* barometer = (baro_str*)barom;
	//boost::asio::io_service io;
    while (true) {
    	//boost::asio::deadline_timer t(io, boost::posix_time::microseconds(2500));
        barometer->_baro.refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer->_baro.readPressure();

        barometer->_baro.refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer->_baro.readTemperature();

        barometer->_baro.calculatePressureAndTemperature();

        barometer->_shmmsg._pres = barometer->_baro.getPressure();
        barometer->_shmmsg._temp = barometer->_baro.getTemperature();
        //sleep(0.5);
        //t.wait();
    }

    pthread_exit(NULL);
}

void * acquireTotalStationData(void * totalStation)
{
	boost::asio::io_service io;
	boost::asio::ip::udp::resolver resolver(io);
	boost::asio::ip::udp::resolver::query query("aegrvc.local", "8000");
	boost::asio::ip::udp::resolver::iterator iter = resolver.resolve(query);

	boost::asio::io_service io_service;
	//create a UDP socket

	boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 8000));

	if (!socket.is_open())
	{
	   printf("Failed to open UDP client socket");
	}else{
		boost::array<char, 1> send_buf  = { 0 };
		//boost::asio::ip::udp::endpoint receiver_endpoint(boost::asio::ip::address::from_string("192.168.1.104"),8000);
		boost::asio::ip::udp::endpoint receiver_endpoint = iter->endpoint();

		socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

		totalStation_str* TS = (totalStation_str*)totalStation;
		UDPTotalStation_msg msg = {0.0,0.0,0.0,0};
		TS->_shmmsg._x=msg._x;
		TS->_shmmsg._y=msg._y;
		TS->_shmmsg._z=msg._z;
		TS->_shmmsg._time=static_cast<double>(msg._time);

		boost::array<char, sizeof(shm_totalStation)> recv_buf;

		//boost::asio::ip::udp::endpoint remote_endpoint(boost::asio::ip::address::from_string("192.168.1.104"),8000);
		boost::asio::ip::udp::endpoint remote_endpoint = iter->endpoint();;
		while (true) {

			if (socket.receive_from(boost::asio::buffer(recv_buf),remote_endpoint) == sizeof(shm_totalStation)){
				memcpy(&msg,&recv_buf[0],sizeof(shm_totalStation));
				if ((msg._x == 0.0)&&(msg._y == 0.0)&&(msg._z == 0.0)){
					TS->_shmmsg._time = 0.0;
				}else{
					TS->_shmmsg._x=msg._x;
					TS->_shmmsg._y=msg._y;
					TS->_shmmsg._z=msg._z;
					TS->_shmmsg._time=static_cast<double>(msg._time);
				}
				recv_buf.assign(0);
				//printf("x: %f| y: %f| z: %f| time: %f\n",TS->_shmmsg._x,TS->_shmmsg._y,TS->_shmmsg._z,TS->_shmmsg._time);
			}
		}
		socket.close();
	}
    pthread_exit(NULL);
}

void * acquireGPSData(void * gps_sig)
{
	boost::asio::io_service io;
	gps_str* gps_data = (gps_str*) gps_sig;
	if(gps_data->_gps.testConnection())
	{
		printf("Ublox test OK\n");
		while(true){
			boost::asio::deadline_timer t(io, boost::posix_time::microseconds(100000));
			if(gps_data->_gps.decodeSingleMessage(Ublox::NAV_POSLLH,gps_data->_pos_data)==1)
			{
				gps_data->_shmmsg._time=gps_data->_pos_data[0]/1000;
				gps_data->_shmmsg._lon=gps_data->_pos_data[1]/10000000;
				gps_data->_shmmsg._lat=gps_data->_pos_data[2]/10000000;
				gps_data->_shmmsg._height_ellipsoid=gps_data->_pos_data[3]/1000;
				gps_data->_shmmsg._height_sea=gps_data->_pos_data[4]/1000;
				gps_data->_shmmsg._hor_accur=gps_data->_pos_data[5]/1000;
				gps_data->_shmmsg._ver_accur=gps_data->_pos_data[6]/1000;
			}
			if (gps_data->_gps.decodeSingleMessage(Ublox::NAV_STATUS,gps_data->_pos_data) == 1){
				gps_data->_shmmsg._status = (int)gps_data->_pos_data[0];
			}
			t.wait();
		}
	}
	pthread_exit(NULL);
}

void * acquirePX4FlowData(void * flow_sig)
{
	boost::asio::io_service io;
	px4flow_str* flow_data = (px4flow_str*) flow_sig;

	px::SerialComm comm(std::string("/px4flow"));

	if (!comm.open(std::string("/dev/ttyAMA0"),115200)){
		printf("ERROR comm: invalid port.\n");
	}else{
		while(true)
		{
			boost::asio::deadline_timer t(io, boost::posix_time::microseconds(10000));

			flow_data->_shmmsg = comm.m_optFlowMsg;
			//printf("Z: %f\n",flow_data->_shmmsg._ground_distance);
			t.wait();
		}
	}
	pthread_exit(NULL);
}

uint32_t read32(uint8_t load[], int index0) {
  return (uint32_t) read16(load,index0) + (uint32_t) (read16(load,index0+2) << 16);
}

uint16_t read16(uint8_t load[], int index0) {
  return load[index0] + (uint16_t) (load[index0+1] << 8);
}

uint8_t read8(uint8_t load[], int index0) {
  return load[index0];
}
double ConvertFrom64toDouble(uint8_t load[], int index0)
{
	//index0 initial position to load;
	uint8_t data[8];
	double f;
	data[0] = load[index0];
	data[1] = load[index0+1];
	data[2] = load[index0+2];
	data[3] = load[index0+3];
	data[4] = load[index0+4];
	data[5] = load[index0+5];
	data[6] = load[index0+6];
	data[7] = load[index0+7];
	f=atof((const char *)data);
	//memcpy(&f,&data,sizeof(f));
	return f;
}



#endif /* DRIVERSNAVIO2_H_ */
