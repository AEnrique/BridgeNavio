/*
 * shm_channels.h
 *
 *  Created on: 1/12/2017
 *      Author: GRVC
 */

#ifndef SHM_CHANNELS_H_
#define SHM_CHANNELS_H_

///SHARED MEMORIES CONSTANTS///
#define SHM_MPU9250              "/shm_mpu9250"            //-R/W imu mpu9250
#define SHM_LSM9DS1              "/shm_lsm9ds1"            //-R/W imu lsm9ds1
#define SHM_MS5611               "/shm_ms5611"		       //-R/W barometer ms5611
#define SHM_RCIN                 "/shm_rcin"               //-R/W Emisor Radio RC input
#define SHM_RCOUT                "/shm_rcout"			   //-R/W Servo output pwm
#define SHM_GPS                  "/shm_gps"                // R/W Gps ublox
#define SHM_ADC                  "/shm_adc"                //-R/W ADC signals
#define SHM_AHRS                 "/shm_ahrs"               //-R/W AHRS signals
#define SHM_SF11C                "/shm_sf11c"              //-R/W SF11C signals
#define SHM_TOTALSTATION         "/shm_totalstation"       //-R/W Leica Total Staion 50 signals
#define SHM_PX4FLOW              "/shm_px4flow"       //-R/W OpticalFlow
//////////////////////////////
/////SEMAPHORES CONSTANTS/////
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
///////////////////////////////


#endif /* SHM_CHANNELS_H_ */
