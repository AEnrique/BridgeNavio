/*
 * bridge_drivers_navio.cpp
 *
 *  Created on: 28/2/2018
 *      Author: GRVC
 */
#include <thread>
#include <shm_util.h>
#include <navio_types.h>
#include <shm_channels.h>
#include <sensors_threads.h>
#include <bridge_drivers_navio.h>
#include <iostream>

#include "global.h"

using namespace std;

	BridgeDriversNavio::BridgeDriversNavio()
	{
		//cout << "Default constructor" << endl;

		_sensor.push_back("mpu");
		create_shm<shm_imu>(&_imu_mpu9250,SHM_MPU9250,SEM_MPU9250);
		_mpu9250_data._mpu9250_imu.initialize();
		_sensor.push_back("rcio");
		create_shm<shm_RCin>(&_rcin,SHM_RCIN,SEM_RCIN);
		create_shm<shm_RCou>(&_rcout,SHM_RCOUT,SEM_RCOUT);
		create_shm<shm_status>(&_status,SHM_STATUS,SEM_STATUS);
		_rcio_data._rcinput.initialize();
		_calib_rotors = false;
		_num_rotors = 4;
	}
	BridgeDriversNavio::BridgeDriversNavio(int argc, char * argv[])
	{
		//cout << "Constructor with arguments" << endl;
		create_shm<shm_status>(&_status,SHM_STATUS,SEM_STATUS);
		_calib_rotors = false;
		_num_rotors=4;
		if (argc > 1){
			_num_rotors=4;

			for (int i=1;i<argc;i++){

				switch (fhash(argv[i])){
					case fhash("mpu"):
						_sensor.push_back("mpu");
						create_shm<shm_imu>(&_imu_mpu9250,SHM_MPU9250,SEM_MPU9250);
						_mpu9250_data._mpu9250_imu.initialize();
						break;
					case fhash("lsm"):
						_sensor.push_back("lsm");
						create_shm<shm_imu>(&_imu_lsm9ds1,SHM_LSM9DS1,SEM_LSM9DS1);
						_lsm9ds1_data._lsm9ds1_imu.initialize();
						break;
					case fhash("baro"):
						_sensor.push_back("baro");
						create_shm<shm_barometer>(&_baro,SHM_MS5611,SEM_MS5611);
						_barometer_data._baro.initialize();
						break;
					case fhash("rcio"):
						_sensor.push_back("rcio");
						create_shm<shm_RCin>(&_rcin,SHM_RCIN,SEM_RCIN);
						create_shm<shm_RCou>(&_rcout,SHM_RCOUT,SEM_RCOUT);
						_rcio_data._rcinput.initialize();
						break;
					case fhash("adc"):
						_sensor.push_back("adc");
						create_shm<shm_adc>(&_adc,SHM_ADC,SEM_ADC);
						_adc_data._adc.initialize();
						break;
					case fhash("gps"):
						_sensor.push_back("gps");
						create_shm<shm_gps>(&_gps,SHM_GPS,SEM_GPS);

						break;
					case fhash("sf11c"):
						_sensor.push_back("sf11c");
						create_shm<shm_lightware>(&_sf11c,SHM_SF11C,SEM_SF11C);

						break;
					case fhash("teraranger"):
						_sensor.push_back("teraranger");
						create_shm<shm_lightware>(&_teraranger,SHM_TERARANGER,SEM_TERARANGER);

						break;
					case fhash("totalstation"):
						_sensor.push_back("totalstation");
						create_shm<shm_totalStation>(&_totalstation,SHM_TOTALSTATION,SEM_TOTALSTATION);

						break;
					case fhash("vicon"):
						_sensor.push_back("vicon");
						create_shm<shm_vicon>(&_vicon,SHM_VICON,SEM_VICON);

						break;
					case fhash("px4flow"):
						_sensor.push_back("px4flow");
						create_shm<shm_px4flow>(&_px4flow,SHM_PX4FLOW,SEM_PX4FLOW);

						break;
					case fhash("svo"):
						_sensor.push_back("svo");
						create_shm<shm_svo>(&_svo,SHM_SVO,SEM_SVO);

						break;
					case fhash("calibrationrotors"):
						_calib_rotors=true;
						break;
					case fhash("quad"):
						_num_rotors=4;
						break;
					case fhash("hexa"):
						_num_rotors=6;
						break;
					case fhash("octo"):
						_num_rotors=8;
						break;
					default:
						break;
				}

			}
		}else{
			_sensor.push_back("mpu");
			create_shm<shm_imu>(&_imu_mpu9250,SHM_MPU9250,SEM_MPU9250);
			_mpu9250_data._mpu9250_imu.initialize();
			_sensor.push_back("rcio");
			create_shm<shm_RCin>(&_rcin,SHM_RCIN,SEM_RCIN);
			create_shm<shm_RCou>(&_rcout,SHM_RCOUT,SEM_RCOUT);
			create_shm<shm_status>(&_status,SHM_STATUS,SEM_STATUS);
			_rcio_data._rcinput.initialize();
			_calib_rotors = false;
			_num_rotors = 4;
		}
	}
	BridgeDriversNavio::~BridgeDriversNavio()
	{
	}

	void BridgeDriversNavio::runThreads()
	{
		_status_data._shmmsg = get_shm<shm_status>(&_status);
		std::thread (acquireSTATUSData,&this->_status_data).detach();
		for(std::string name_sensor: _sensor){
			switch(fhash(name_sensor.c_str())){
				case fhash("mpu"):
					if (set_shm<shm_imu>(&_imu_mpu9250, _mpu9250_data._shmmsg) == 0){
						cout << "imu_mpu9250:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireMPU9250Data,&this->_mpu9250_data).detach();
					}
					break;

				case fhash("lsm"):
					if (set_shm<shm_imu>(&_imu_lsm9ds1, _lsm9ds1_data._shmmsg) == 0){
						cout << "imu_lsm9ds1:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireLSM9DS1Data,&this->_lsm9ds1_data).detach();
					}
					break;

				case fhash("baro"):
					if (set_shm<shm_barometer>(&_baro, _barometer_data._shmmsg) == 0){
						cout << "shm_barometer:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireBarometerData,&this->_barometer_data).detach();
					}
					break;

				case fhash("rcio"):
					_rcio_data._shmmsg_rcout = get_shm<shm_RCou>(&_rcout);
					if (set_shm<shm_RCin>(&_rcin, _rcio_data._shmmsg_rcin) == 0)
						cout << "shm_RCin:Error to initializate the shared memory direction.\n" << endl;
					std::thread (acquireSendRCIOData,&(this->_rcio_data),this->_num_rotors).detach();
					break;

				case fhash("adc"):
					if (set_shm<shm_adc>(&_adc, _adc_data._shmmsg) == 0){
						cout << "shm_adc:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireADCData,&this->_adc_data).detach();
					}
					break;

				case fhash("gps"):
					if (set_shm<shm_gps>(&_gps, _gps_data._shmmsg) == 0){
						cout << "shm_gps:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireGPSData,&this->_gps_data).detach();
					}
					break;

				case fhash("sf11c"):
					if (set_shm<shm_lightware>(&_sf11c, _sf11c_data._shmmsg) == 0){
						cout << "shm_lightware:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireSF11CData,&this->_sf11c_data).detach();
					}
					break;

				case fhash("teraranger"):
					if (set_shm<shm_lightware>(&_teraranger, _teraranger_data._shmmsg) == 0){
						cout << "shm_lightware:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireTERARANGERData,&this->_teraranger_data).detach();
					}
					break;
				case fhash("totalstation"):
					if (set_shm<shm_totalStation>(&_totalstation, _totalstation_data._shmmsg) == 0){
						cout << "shm_totalStation:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireTotalStationData,&this->_totalstation_data).detach();
					}
					break;
				case fhash("vicon"):
					if (set_shm<shm_vicon>(&_vicon, _vicon_data._shmmsg) == 0){
						cout << "shm_vicon:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireVICONData,&this->_vicon_data).detach();
					}
					break;
				case fhash("px4flow"):
					if (set_shm<shm_px4flow>(&_px4flow, _px4flow_data._shmmsg) == 0){
						cout << "shm_px4flow:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquirePX4FlowData,&this->_px4flow_data).detach();
					}
					break;
				case fhash("svo"):
					if (set_shm<shm_svo>(&_svo, _svo_data._shmmsg) == 0){
						cout << "shm_svo:Error to initializate the shared memory direction.\n" << endl;
					}else{
						std::thread (acquireSVOData,&this->_svo_data).detach();
					}
					break;
				default:
					cout << "Error: No thread has been released\n" << endl;
					break;

			}

		}


	}
	void BridgeDriversNavio::close(int sig )
	{
		try
		{
			cout << "Closing all!" << endl;
        	if (close_shm(&_imu_mpu9250) == -1)
        		cout << "mpu9250:Error closing shared memory.\n" << endl;

        	if (close_shm(&_imu_lsm9ds1) == -1)
        		cout << "lsm9ds1:Error closing shared memory.\n" << endl;

        	if (close_shm(&_rcin) == -1)
        		cout << "RCin:Error closing shared memory.\n" << endl;

        	if (close_shm(&_adc) == -1)
        		cout << "ADC:Error closing shared memory.\n" << endl;

        	if (close_shm(&_baro) == -1)
        		cout << "Baro:Error closing shared memory.\n" << endl;

        	if (close_shm(&_rcout) == -1)
        		cout << "rcout:Error closing shared memory.\n" << endl;

           	if (close_shm(&_sf11c) == -1)
           		cout << "sf11c:Error closing shared memory.\n" << endl;

           	if (close_shm(&_teraranger) == -1)
           		cout << "teraranger:Error closing shared memory.\n" << endl;

           	if (close_shm(&_gps) == -1)
           		cout << "gps:Error closing shared memory.\n" << endl;

          	if (close_shm(&_totalstation) == -1)
          		cout << "TotalStation:Error closing shared memory.\n" << endl;

          	if (close_shm(&_vicon) == -1)
          		cout << "VICON:Error closing shared memory.\n" << endl;

         	if (close_shm(&_px4flow) == -1)
         		cout << "Px4Flow:Error closing shared memory.\n" << endl;

         	if (close_shm(&_svo) == -1)
         	    cout << "SVO:Error closing shared memory.\n" << endl;

         	if (close_shm(&_status) == -1)
         	    cout << "STATUS:Error closing shared memory.\n" << endl;
         	std::terminate();
		}catch(int error){

		}
	}

	void BridgeDriversNavio::setShm()
	{
		try
		{
			signal(SIGINT,BridgeDriversNavio::close);
			_status_data._shmmsg = get_shm<shm_status>(&_status);
			for(std::string name_sensor: _sensor){
				switch(fhash(name_sensor.c_str())){
					case fhash("mpu"):
						if (set_shm<shm_imu>(&_imu_mpu9250, _mpu9250_data._shmmsg) == 0){
							cout << "imu_mpu9250:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("lsm"):
						if (set_shm<shm_imu>(&_imu_lsm9ds1, _lsm9ds1_data._shmmsg) == 0){
							cout << "imu_lsm9ds1:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("baro"):
						if (set_shm<shm_barometer>(&_baro, _barometer_data._shmmsg) == 0){
							cout << "shm_barometer:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("rcio"):
						_rcio_data._shmmsg_rcout = get_shm<shm_RCou>(&_rcout);
						if (set_shm<shm_RCin>(&_rcin, _rcio_data._shmmsg_rcin) == 0)
							cout << "shm_RCin:Error to write in shared memory direction.\n" << endl;
						break;

					case fhash("adc"):
						if (set_shm<shm_adc>(&_adc, _adc_data._shmmsg) == 0){
							cout << "shm_adc:Error to write in shared memory direction.\n" << endl;
						}

						break;

					case fhash("gps"):
						if (set_shm<shm_gps>(&_gps, _gps_data._shmmsg) == 0){
							cout << "shm_gps:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("sf11c"):
						if (set_shm<shm_lightware>(&_sf11c, _sf11c_data._shmmsg) == 0){
							cout << "shm_lightware:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("teraranger"):
						if (set_shm<shm_lightware>(&_teraranger, _teraranger_data._shmmsg) == 0){
							cout << "shm_lightware:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("totalstation"):
						if (set_shm<shm_totalStation>(&_totalstation, _totalstation_data._shmmsg) == 0){
							cout << "shm_totalStation:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("vicon"):
						if (set_shm<shm_vicon>(&_vicon, _vicon_data._shmmsg) == 0){
							cout << "shm_vicon:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("px4flow"):
						if (set_shm<shm_px4flow>(&_px4flow, _px4flow_data._shmmsg) == 0){
							cout << "shm_px4flow:Error to write in shared memory direction.\n" << endl;
						}
						break;

					case fhash("svo"):
						if (set_shm<shm_svo>(&_svo, _svo_data._shmmsg) == 0){
							cout << "shm_svo:Error to write in the shared memory direction.\n" << endl;
						}
						break;

					default:
						cout << "ERROR: Command for sensor not found!.\n" << endl;
						break;

				}

			}

		}catch(int error){}

	}


