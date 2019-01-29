/*
 * global.h
 *
 *  Created on: 29 ago. 2018
 *      Author: ae-grvc
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <shm_util.h>
#include <navio_types.h>

static struct shm_str<shm_imu> _imu_mpu9250;
static struct shm_str<shm_imu> _imu_lsm9ds1;
static struct shm_str<shm_RCin> _rcin;
static struct shm_str<shm_barometer> _baro;
static struct shm_str<shm_RCou> _rcout;
static struct shm_str<shm_adc> _adc;
static struct shm_str<shm_lightware> _sf11c;
static struct shm_str<shm_totalStation> _totalstation;
static struct shm_str<shm_gps> _gps;
static struct shm_str<shm_px4flow> _px4flow;
static struct shm_str<shm_svo> _svo;
static struct shm_str<shm_status> _status;
static struct shm_str<shm_vicon> _vicon;
static struct shm_str<shm_lightware> _teraranger;

#endif /* GLOBAL_H_ */
