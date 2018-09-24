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

#ifndef SENSORS_THREADS_H_
#define SENSORS_THREADS_H_

#include <thread>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <boost/array.hpp>
#include <sstream>

////////////////////////////
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<stdint.h>
#include<arpa/inet.h>
#include<sys/socket.h>
////////////////////////////

#include <bridge_drivers_navio.h>
#include <serial_comm.h>

//------------------------------THREADS SENSORS FUNCTIONS------------------------------//

/// Used to manage RC input and ESC PWM output
	void acquireSendRCIOData(rcio_str * rc,int8_t n_rotors)
	{
		printf("RCIO thread launched\n");
		boost::asio::io_service io;

		float pwm_out[n_rotors];
		int i;

		for(i = 0;i < n_rotors; i++)
		{
			pwm_out[i] = SERVO_MIN;
			rc->_pwm.initialize(i);
			rc->_pwm.set_frequency(i, 400);
			rc->_pwm.enable(i);
		}
		if (rc->_calib_rotors_required){
			printf("Calibration of ESC in process...\n");
			for(i = 0;i < n_rotors; i++)
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


	    	switch (n_rotors){
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
	    			//printf("Rotors %d\n", n_rotors);
	    	    	break;
	    	}


	    	for(i = 0;i < n_rotors; i++)
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

	    }
	    std::terminate();
	}
//-----------------------------------------------------------------------------------------//

/// Lightware altimeter SF11/C driver
	void acquireSF11CData(sf11c_str * sf11c_data)
	{
		printf("SF11C thread launched\n");

		boost::asio::io_service io;
		int fd;
		unsigned char buffer_sf11c[2];
		if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)//with Grove cape i2c-2 is i2c-1 and vice.
		{
	    // Open port for reading and writing
			fprintf(stderr, "Failed to open i2c bus\n");
			exit(1);
		}else{
			while (true){
				boost::asio::deadline_timer t(io, boost::posix_time::microseconds(50000));
				if (ioctl(fd, I2C_SLAVE, 0x66) < 0) {
					printf("Failed to acquire bus access and/or talk to slave.\n");
					//ERROR HANDLING; you can check errno to see what went wrong
					exit(1);
				}else{
					if (read(fd, buffer_sf11c, 2) != 2) {
						printf("Unable to read from SF11C\r");
						fflush(stdout);
					} else {
						sf11c_data->_shmmsg._z = float(((uint16_t)buffer_sf11c[0] << 8) | buffer_sf11c[1])*1e-2f;

					}
				}
			t.wait();
			}
		}

	}
//-----------------------------------------------------------------------------------------//

///ADC
	void acquireADCData(adc_str * adc_data)
	{
		printf("ADC thread launched\n");
		boost::asio::io_service io;
	    float results[adc_data->_adc.get_channel_count()] = {0.0f};
	    while (true) {
	    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(500000));
	        for (int i = 0; i < adc_data->_adc.get_channel_count(); i++)
	        {
	            results[i] = adc_data->_adc.read(i);
	        }
	        adc_data->_shmmsg._adc0 = results[0];
	        adc_data->_shmmsg._adc1 = results[1];
	        adc_data->_shmmsg._adc2 = results[2];
	        adc_data->_shmmsg._adc3 = results[3];
	        adc_data->_shmmsg._adc4 = results[4];
	        adc_data->_shmmsg._adc5 = results[5];
	        t.wait();
	    }


	}
//-----------------------------------------------------------------------------------------//

/// LSM9DS1
	void acquireLSM9DS1Data(lsm9ds1_imu_str * imu)
	{
		printf("LSM9DS1 thread launched\n");
		boost::asio::io_service io;
	    while (true) {
	    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(2500));
	    	imu->_lsm9ds1_imu.update();
	    	imu->_lsm9ds1_imu.read_accelerometer(&imu->_shmmsg._ax,&imu->_shmmsg._ay, &imu->_shmmsg._az);
	    	imu->_lsm9ds1_imu.read_gyroscope(&imu->_shmmsg._gx, &imu->_shmmsg._gy, &imu->_shmmsg._gz);
	    	imu->_lsm9ds1_imu.read_magnetometer(&imu->_shmmsg._mx, &imu->_shmmsg._my, &imu->_shmmsg._mz);
	    	imu->_shmmsg._temperature = imu->_lsm9ds1_imu.read_temperature();//~0.000750s
			t.wait();
	    }


	}
//-----------------------------------------------------------------------------------------//

/// MPU9250
	void acquireMPU9250Data(mpu9250_imu_str * imu)
	{
		printf("MPU9250 thread launched\n");
		boost::asio::io_service io;
	    while (true) {
	    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(2500));
	    	imu->_mpu9250_imu.update();
	    	imu->_mpu9250_imu.read_accelerometer(&imu->_shmmsg._ax, &imu->_shmmsg._ay, &imu->_shmmsg._az);
	    	imu->_mpu9250_imu.read_gyroscope(&imu->_shmmsg._gx, &imu->_shmmsg._gy, &imu->_shmmsg._gz);
	    	imu->_mpu9250_imu.read_magnetometer(&imu->_shmmsg._mx, &imu->_shmmsg._my, &imu->_shmmsg._mz);
	    	imu->_shmmsg._temperature = imu->_mpu9250_imu.read_temperature();//~0.000450s
	        t.wait();


	    }

	}
//-----------------------------------------------------------------------------------------//

/// Barometer
	void acquireBarometerData(baro_str * barometer)
	{
		printf("Barometer thread launched\n");
		//baro_str* barometer = (baro_str*)barom;
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


	}
//-----------------------------------------------------------------------------------------//

/// TotalStation client
	void acquireTotalStationData(totalStation_str * totalStation)
	{
		printf("Total Station thread launched\n");
		boost::asio::io_service io;
		boost::asio::ip::udp::resolver resolver(io);
		boost::asio::ip::udp::resolver::query query(PC_IP, UDP_PORT_TOTALSTATION); //UDP_PORT
		boost::asio::ip::udp::resolver::iterator iter = resolver.resolve(query);

		/*std::stringstream strValue;
		strValue << UDP_PORT;
		unsigned int intValue;
		strValue >> intValue;*/

		boost::asio::io_service io_service;
		//create a UDP socket

		boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), c_uint(UDP_PORT_TOTALSTATION)));

		if (!socket.is_open())
		{
		   printf("Failed to open UDP client socket");
		}else{
			boost::array<char, 1> send_buf  = { 0 };

			boost::asio::ip::udp::endpoint receiver_endpoint = iter->endpoint();

			socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

			UDPTotalStation_msg msg = {0.0,0.0,0.0,0};
			totalStation->_shmmsg._x=msg._x;
			totalStation->_shmmsg._y=msg._y;
			totalStation->_shmmsg._z=msg._z;
			totalStation->_shmmsg._time=static_cast<double>(msg._time);

			boost::array<char, sizeof(shm_totalStation)> recv_buf;


			boost::asio::ip::udp::endpoint remote_endpoint = iter->endpoint();;
			while (true) {

				if (socket.receive_from(boost::asio::buffer(recv_buf),remote_endpoint) == sizeof(shm_totalStation)){
					memcpy(&msg,&recv_buf[0],sizeof(shm_totalStation));
					if ((msg._x == 0.0)&&(msg._y == 0.0)&&(msg._z == 0.0)){
						totalStation->_shmmsg._time = 0.0;
					}else{
						totalStation->_shmmsg._x=msg._x;
						totalStation->_shmmsg._y=msg._y;
						totalStation->_shmmsg._z=msg._z;
						totalStation->_shmmsg._time=static_cast<double>(msg._time);
					}
					recv_buf.assign(0);
					//printf("x: %f| y: %f| z: %f| time: %f\n",TS->_shmmsg._x,TS->_shmmsg._y,TS->_shmmsg._z,TS->_shmmsg._time);
				}
			}
			socket.close();
		}

	}
//-----------------------------------------------------------------------------------------//

/// GPS
	void acquireGPSData(gps_str * gps_data)
	{
		printf("GPS thread launched\n");
		boost::asio::io_service io;

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

	}
//-----------------------------------------------------------------------------------------//

/// PX4Flow Driver
	void acquirePX4FlowData(px4flow_str * flow_data)
	{
		printf("PX4Flow thread launched\n");
		boost::asio::io_service io;

		px::SerialComm comm(std::string("/px4flow"));

		if (!comm.open(std::string("/dev/ttyAMA0"),115200)){
			printf("ERROR comm: invalid port.\n");
		}else{
			while(true)
			{
				boost::asio::deadline_timer t(io, boost::posix_time::microseconds(10000));

				flow_data->_shmmsg = comm.m_optFlowMsg;

				t.wait();
			}
		}

	}
//-----------------------------------------------------------------------------------------//


/// Receive SVO algorithm data send by udp from odroid
	void acquireSVOData(svo_str * svo)
	{
		printf("SVO received thread launched\n");
		boost::asio::io_service io;
		boost::asio::ip::udp::resolver resolver(io);
		boost::asio::ip::udp::resolver::query query(ODROID_IP, UDP_PORT_ODROID); //UDP_PORT
		boost::asio::ip::udp::resolver::iterator iter = resolver.resolve(query);


		boost::asio::io_service io_service;
		//create a UDP socket

		boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), c_uint(UDP_PORT_ODROID)));

		if (!socket.is_open())
		{
			printf("Failed to open UDP client socket\n");
		}else{
			printf("UDP client socket opened\n");
			boost::array<char, 1> send_buf  = { 0 };

			boost::asio::ip::udp::endpoint receiver_endpoint = iter->endpoint();

			socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

			UDPSVO_msg msg = {0,0,0.0,0.0,0.0,{0.0,0.0,0.0,1.0}};
			svo->_shmmsg._x=msg._x_s;
			svo->_shmmsg._y=msg._y_s;
			svo->_shmmsg._z=msg._z_s;
			svo->_shmmsg._qx=msg._quat[0];
			svo->_shmmsg._qy=msg._quat[1];
			svo->_shmmsg._qz=msg._quat[2];
			svo->_shmmsg._qw=msg._quat[3];


			boost::array<char, sizeof(UDPSVO_msg)> recv_buf;


			boost::asio::ip::udp::endpoint remote_endpoint = iter->endpoint();;
			while (true) {

				if (socket.receive_from(boost::asio::buffer(recv_buf),remote_endpoint) == sizeof(UDPSVO_msg)){
					memcpy(&msg,&recv_buf[0],sizeof(UDPSVO_msg));
					svo->_shmmsg._x=msg._x_s;
					svo->_shmmsg._y=msg._y_s;
					svo->_shmmsg._z=msg._z_s;
					svo->_shmmsg._qx=msg._quat[0];
					svo->_shmmsg._qy=msg._quat[1];
					svo->_shmmsg._qz=msg._quat[2];
					svo->_shmmsg._qw=msg._quat[3];
					recv_buf.assign(0);
					//printf("1 SVO -> x: %f| y: %f| z: %f\n",svo->_shmmsg._x,svo->_shmmsg._y,svo->_shmmsg._z);
				}else{
					printf("Error receive_from\n");
				}
			}
			socket.close();
		}

	}
//-----------------------------------------------------------------------------------------//


#endif /* SENSORS_THREADS_H_ */
