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

#ifndef SHM_CHANNELS_H_
#define SHM_CHANNELS_H_

///SHARED MEMORIES CONSTANTS
#define SHM_MPU9250              "/shm_mpu9250"            		//-R/W imu mpu9250
#define SHM_LSM9DS1              "/shm_lsm9ds1"            		//-R/W imu lsm9ds1
#define SHM_MS5611               "/shm_ms5611"		       		//-R/W barometer ms5611
#define SHM_RCIN                 "/shm_rcin"               		//-R/W Emisor Radio RC input
#define SHM_RCOUT                "/shm_rcout"			   		//-R/W Servo output pwm
#define SHM_GPS                  "/shm_gps"                		// R/W Gps ublox
#define SHM_ADC                  "/shm_adc"                		//-R/W ADC signals
#define SHM_AHRS                 "/shm_ahrs"               		//-R/W AHRS signals
#define SHM_SF11C                "/shm_sf11c"              		//-R/W SF11C signals
#define SHM_TOTALSTATION         "/shm_totalstation"       		//-R/W Leica Total Station 50 signals
#define SHM_PX4FLOW              "/shm_px4flow"       	   		//-R/W OpticalFlow
#define SHM_SVO              	 "/shm_svo"       		   		//-R/W SVO algorithm signals
#define SHM_ATTPX4               "/shm_attitudePX4EKF2"         //-R/W attitude ekf2 px4
#define SHM_GLOBALPOSPX4         "/shm_globalPositionPX4EKF2"   //-R/W local position ekf2 px4
#define SHM_LOCALPOSPX4          "/shm_localPositionPX4EKF2"   //-R/W global position ekf2 px4
#define SHM_STATUS               "/shm_status"   //-R/W status

///SEMAPHORES CONSTANTS
#define SEM_MPU9250              "/mysem_mpu9250"
#define SEM_LSM9DS1              "/mysem_lsm9ds1"
#define SEM_MS5611               "/mysem_ms5611"
#define SEM_RCIN                 "/mysem_rcin"
#define SEM_RCOUT                "/mysem_rcout"
#define SEM_GPS                  "/mysem_gps"
#define SEM_ADC                  "/mysem_adc"
#define SEM_AHRS                 "/mysem_ahrs"
#define SEM_SF11C                "/mysem_sf11c"
#define SEM_TOTALSTATION         "/mysem_totalstation"
#define SEM_PX4FLOW              "/mysem_px4flow"
#define SEM_SVO              	 "/mysem_svo"
#define SEM_ATTPX4               "/mysem_attitudePX4EKF2"         //-R/W attitude ekf2 px4
#define SEM_GLOBALPOSPX4         "/mysem_globalPositionPX4EKF2"   //-R/W local position ekf2 px4
#define SEM_LOCALPOSPX4          "/mysem_localPositionPX4EKF2"   //-R/W global position ekf2 px4
#define SEM_STATUS               "/mysem_status"   //-R/W status
///////////////////////////////


#endif /* SHM_CHANNELS_H_ */
